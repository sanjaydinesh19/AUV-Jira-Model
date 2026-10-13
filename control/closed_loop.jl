# Closed-loop simulation: pitch hold + heading hold + fin allocator on the model.
#
# The controller runs at 20 Hz with a zero-order hold: every 50 ms it reads
# θ, θ̇, ψ, ψ̇ (the only quantities the Pixhawk IMU measures), computes new fin
# angles, and the model is then integrated for 50 ms with those fins fixed.
# Thrust is constant for the whole run.
using OrdinaryDiffEq
using ADTypes: AutoFiniteDiff

include(joinpath(@__DIR__, "..", "src", "auv_model.jl"))
include(joinpath(@__DIR__, "config.jl"))
include(joinpath(@__DIR__, "pid.jl"))
include(joinpath(@__DIR__, "allocator.jl"))

const A = AUVModel
const SIM_SOLVER = AutoTsit5(Rosenbrock23(autodiff = AutoFiniteDiff()))

integrate_model(u, tspan, p) =
    solve(ODEProblem(A.dyn!, u, tspan, p), SIM_SOLVER;
          reltol = SIM_RELTOL, abstol = SIM_ABSTOL,
          save_everystep = false, maxiters = 5_000_000).u[end]

# Terminal forward speed for a constant thrust with fins centred. This is a
# MODEL PREDICTION (there is no speed sensor); it is what the Pi will use to
# schedule the allocator.
function nominal_speed(thrust_N; p_base = A.p_vals)
    p = copy(p_base)
    p[A.idx_T] = thrust_N
    return integrate_model(zeros(8), (0.0, 40.0), p)[5]
end

no_disturbance!(p, t_s) = nothing

"""
    simulate_closed_loop(; thrust_N, t_end_s, pitch_ref, yaw_ref, ...)

`pitch_ref(t)` and `yaw_ref(t)` return the references in rad (model signs:
θ positive nose DOWN, ψ positive counter-clockwise from above).

`speed_source` chooses the speed the allocator is evaluated at:
  :true_state  the model's own state (ideal; needs a speed sensor we do not have)
  :nominal     level hull at the nominal speed for this thrust (what the Pi does)

`disturbance!(p, t)` may change the parameter vector during the run.
"""
function simulate_closed_loop(; thrust_N, t_end_s, pitch_ref, yaw_ref,
                              u0 = nothing, p_base = A.p_vals,
                              speed_source = :true_state,
                              disturbance! = no_disturbance!)
    dt_s = CONTROL_DT_S
    p = copy(p_base)
    p[A.idx_T] = thrust_N
    fin_idx = [A.idx_δ1, A.idx_δ2, A.idx_δ3]
    p[fin_idx] .= 0.0

    speed_nominal_m_s = nominal_speed(thrust_N; p_base = p_base)
    # Default start: level, already moving at the nominal speed.
    u = u0 === nothing ? [0.0, 0.0, 0.0, 0.0, speed_nominal_m_s, 0.0, 0.0, 0.0] : copy(u0)

    pitch_pid = PID(PITCH_KP, PITCH_KI, PITCH_KD, PITCH_INTEGRAL_LIMIT)
    yaw_pid   = PID(YAW_KP, YAW_KI, YAW_KD, YAW_INTEGRAL_LIMIT)
    fins_rad  = zeros(3)
    saturated = false

    n_steps = round(Int, t_end_s / dt_s)
    log = (t = zeros(n_steps), state = zeros(n_steps, 8), fins = zeros(n_steps, 3),
           pitch_ref = zeros(n_steps), yaw_ref = zeros(n_steps),
           low_speed = falses(n_steps))

    for k in 1:n_steps
        t_s = (k - 1) * dt_s
        disturbance!(p, t_s)

        # --- what the IMU gives us ---
        θ, ψ, θ_rate, ψ_rate = u[3], u[4], u[7], u[8]

        speed_m_s = speed_source == :nominal ? speed_nominal_m_s : u[5]
        low_speed = speed_m_s < MIN_CONTROL_SPEED_M_S

        # --- PIDs (integral frozen while saturated or without fin authority) ---
        integrate = !saturated && !low_speed
        pitch_moment_Nm = pid_update!(pitch_pid, pitch_ref(t_s) - θ, θ_rate, dt_s;
                                      integrate = integrate)
        yaw_moment_Nm   = pid_update!(yaw_pid, wrap_pi(yaw_ref(t_s) - ψ), ψ_rate, dt_s;
                                      integrate = integrate)

        # --- allocator ---
        if low_speed
            fin_cmd_rad = zeros(3)          # no authority: neutral fins, do not saturate
        else
            u_sched = speed_source == :nominal ?
                      [0.0, 0.0, 0.0, 0.0, speed_m_s, 0.0, 0.0, 0.0] : u
            Bf = fin_block(A.B_fun(u_sched, p))
            fin_cmd_rad = allocate_fins(Bf, pitch_moment_Nm, yaw_moment_Nm)
        end
        saturated = any(abs.(fin_cmd_rad) .> FIN_LIMIT_RAD)
        fins_rad  = limit_fins(fin_cmd_rad, fins_rad, dt_s)
        p[fin_idx] .= fins_rad

        log.t[k] = t_s
        log.state[k, :] .= u
        log.fins[k, :] .= fins_rad
        log.pitch_ref[k] = pitch_ref(t_s)
        log.yaw_ref[k] = yaw_ref(t_s)
        log.low_speed[k] = low_speed

        # --- plant: 50 ms with the fins held ---
        u = integrate_model(u, (t_s, t_s + dt_s), p)
    end
    return log
end

# Step-response numbers for a step applied at t_step_s.
#   overshoot      : how far past the reference, as % of the step size
#   settling time  : time after the step until the signal stays within
#                    ±band of the reference (NaN if it never does)
#   steady error   : mean (reference - signal) over the last 2 s
function step_metrics(t, y, y_ref_after; t_step_s, band_fraction = 0.05)
    i_step = findfirst(>=(t_step_s), t)
    y_start = y[i_step]
    step = y_ref_after - y_start
    after = i_step:length(t)

    past_ref = sign(step) .* (y[after] .- y_ref_after)
    overshoot_pct = 100 * max(maximum(past_ref), 0.0) / abs(step)

    band = band_fraction * abs(step)
    outside = findlast(i -> abs(y[i] - y_ref_after) > band, after)
    settling_time_s = outside === nothing ? 0.0 :
                      outside == length(after) ? NaN :
                      t[after[outside] + 1] - t_step_s

    last_2s = t .>= t[end] - 2.0
    steady_error = sum(y_ref_after .- y[last_2s]) / count(last_2s)

    return (; overshoot_pct, settling_time_s, steady_error)
end

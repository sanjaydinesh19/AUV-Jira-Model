# Closed-loop test scenarios: CLAUDE_AUV_PJT.md Section 6, Step 1.
# Prints a metrics table and writes it to analysis/closed_loop_metrics.csv.
#
# Run from the repo root:  julia --project=. control/run_closed_loop_tests.jl
#
# Every number printed here is a SIMULATION result on a model with placeholder
# parameters. Nothing here is a measurement.
using Test
using Printf

include(joinpath(@__DIR__, "closed_loop.jl"))

const T_STEP_S = 15.0     # the first 15 s hold θ_ref = ψ_ref = 0, then the step
const T_END_S  = 40.0

step_at(t_step_s, value) = t -> t < t_step_s ? 0.0 : value
zero_ref(t) = 0.0

results = NamedTuple[]

function report!(name, log, signal, ref_after; t_step_s = T_STEP_S, note = "")
    y = signal == :pitch ? log.state[:, 3] : log.state[:, 4]
    m = step_metrics(log.t, y, ref_after; t_step_s = t_step_s)
    max_fin_deg = rad2deg(maximum(abs.(log.fins)))
    push!(results, (; name, signal, step_deg = rad2deg(ref_after - y[findfirst(>=(t_step_s), log.t)]),
                    overshoot_pct = m.overshoot_pct, settling_time_s = m.settling_time_s,
                    steady_error_deg = rad2deg(m.steady_error), max_fin_deg, note))
    return m
end

@testset "closed-loop simulation (Step 1)" begin

    @testset "hold θ_ref = 0 against the natural trim" begin
        # Fins centred the hull settles at θ ≈ +0.091 rad. Start there, switch
        # the controller on, and the integrator must bring θ back to 0.
        u_trim = [0.0, 0.0, 0.0911, 0.0, 0.8289, -0.0138, 0.0, 0.0]
        log = simulate_closed_loop(thrust_N = 8.0, t_end_s = 30.0, u0 = u_trim,
                                   pitch_ref = zero_ref, yaw_ref = zero_ref)
        m = report!("trim cancel, T=8", log, :pitch, 0.0; t_step_s = 0.0)
        @test abs(m.steady_error) < deg2rad(0.2)
    end

    @testset "pitch steps ±5°, T = 8 N" begin
        for step_deg in (-5.0, 5.0)
            log = simulate_closed_loop(thrust_N = 8.0, t_end_s = T_END_S,
                                       pitch_ref = step_at(T_STEP_S, deg2rad(step_deg)),
                                       yaw_ref = zero_ref)
            m = report!(@sprintf("pitch step %+.0f°, T=8", step_deg), log, :pitch,
                        deg2rad(step_deg))
            @test abs(m.steady_error) < deg2rad(0.2)
            @test m.overshoot_pct < 30
            @test maximum(abs.(log.state[:, 4])) < 1e-6      # no yaw excited
        end
    end

    @testset "heading step +10°, T = 8 N (short window, no-sway model)" begin
        log = simulate_closed_loop(thrust_N = 8.0, t_end_s = T_END_S,
                                   pitch_ref = zero_ref,
                                   yaw_ref = step_at(T_STEP_S, deg2rad(10.0)))
        m = report!("heading step +10°, T=8", log, :yaw, deg2rad(10.0);
                    note = "yaw is an approximation (no sway DOF)")
        @test abs(m.steady_error) < deg2rad(0.5)
    end

    @testset "speed sweep: pitch step -5° at T = 5, 12 N" begin
        for thrust_N in (5.0, 12.0)
            log = simulate_closed_loop(thrust_N = thrust_N, t_end_s = T_END_S,
                                       pitch_ref = step_at(T_STEP_S, deg2rad(-5.0)),
                                       yaw_ref = zero_ref)
            m = report!(@sprintf("pitch step -5°, T=%.0f", thrust_N), log, :pitch,
                        deg2rad(-5.0))
            @test abs(m.steady_error) < deg2rad(0.3)
        end
    end

    @testset "allocator scheduled on nominal speed (what the Pi does)" begin
        log = simulate_closed_loop(thrust_N = 8.0, t_end_s = T_END_S,
                                   pitch_ref = step_at(T_STEP_S, deg2rad(-5.0)),
                                   yaw_ref = zero_ref, speed_source = :nominal)
        m = report!("pitch step -5°, T=8, nominal speed", log, :pitch, deg2rad(-5.0))
        @test abs(m.steady_error) < deg2rad(0.2)
    end

    @testset "disturbance: buoyancy ±2% at t = 15 s" begin
        idx_Bb = A.pidx(:Bb)
        for change in (0.02, -0.02)
            Bb_new = A.p_vals[idx_Bb] * (1 + change)
            disturb!(p, t_s) = (t_s >= T_STEP_S && (p[idx_Bb] = Bb_new); nothing)
            log = simulate_closed_loop(thrust_N = 8.0, t_end_s = T_END_S,
                                       pitch_ref = zero_ref, yaw_ref = zero_ref,
                                       disturbance! = disturb!)
            after = log.t .>= T_STEP_S
            peak_deg = rad2deg(maximum(abs.(log.state[after, 3])))
            # Not a reference step, so overshoot and settling time do not apply.
            last_2s = log.t .>= log.t[end] - 2.0
            steady_error = -sum(log.state[last_2s, 3]) / count(last_2s)
            push!(results, (; name = @sprintf("Bb %+.0f%% at 15 s", 100 * change),
                            signal = :pitch, step_deg = 0.0, overshoot_pct = NaN,
                            settling_time_s = NaN, steady_error_deg = rad2deg(steady_error),
                            max_fin_deg = rad2deg(maximum(abs.(log.fins))),
                            note = @sprintf("disturbance: peak pitch error %.2f°", peak_deg)))
            @test abs(steady_error) < deg2rad(0.2)
        end
    end

    @testset "fin saturation: ask for -25° pitch, then back to 0" begin
        # -25° is more than the fins can hold. The fins must sit on the limit
        # without the integral winding up, then recover cleanly.
        ref(t) = 15.0 <= t < 30.0 ? deg2rad(-25.0) : 0.0
        log = simulate_closed_loop(thrust_N = 8.0, t_end_s = 50.0,
                                   pitch_ref = ref, yaw_ref = zero_ref)
        m = report!("saturation recovery (-25° → 0)", log, :pitch, 0.0; t_step_s = 30.0,
                    note = "metrics are for the return step")
        @test maximum(abs.(log.fins)) <= FIN_LIMIT_RAD + 1e-12
        @test all(isfinite, log.state)
        @test abs(m.steady_error) < deg2rad(0.2)
    end

    @testset "low speed: must degrade gracefully, not blow up" begin
        # From rest. At T = 0.5 N the hull never reaches the minimum control
        # speed, so the fins must stay neutral. At T = 2 N it starts below the
        # minimum speed and gains authority on the way.
        for thrust_N in (0.5, 2.0)
            log = simulate_closed_loop(thrust_N = thrust_N, t_end_s = T_END_S,
                                       u0 = zeros(8), pitch_ref = zero_ref,
                                       yaw_ref = zero_ref)
            @test all(isfinite, log.state)
            @test maximum(abs.(log.state[:, 3])) < 0.6      # below the pitch abort limit
            @test maximum(abs.(log.fins)) <= FIN_LIMIT_RAD + 1e-12
            if thrust_N == 0.5
                @test all(log.low_speed) && all(iszero, log.fins)
            end
            @printf("low speed, T=%.1f N: final speed %.3f m/s, final pitch %+.2f°, max |fin| %.1f°, neutral-fin time %.1f s\n",
                    thrust_N, log.state[end, 5], rad2deg(log.state[end, 3]),
                    rad2deg(maximum(abs.(log.fins))), count(log.low_speed) * CONTROL_DT_S)
        end
    end
end

# ---------------------------------------------------------------- report ----
println()
println("SIMULATION results, placeholder model parameters (settling band = ±5% of the step)")
@printf("%-36s %8s %10s %10s %12s %9s\n", "scenario", "step", "overshoot", "settling", "steady err", "max fin")
for r in results
    @printf("%-36s %7.1f° %9.1f%% %9.2fs %11.3f° %8.1f°  %s\n", r.name, r.step_deg,
            r.overshoot_pct, r.settling_time_s, r.steady_error_deg, r.max_fin_deg, r.note)
end

out_path = joinpath(@__DIR__, "..", "analysis", "closed_loop_metrics.csv")
mkpath(dirname(out_path))
open(out_path, "w") do io
    println(io, "scenario,signal,step_deg,overshoot_pct,settling_time_s,steady_error_deg,max_fin_deg,note")
    for r in results
        @printf(io, "\"%s\",%s,%.3f,%.2f,%.2f,%.4f,%.2f,\"%s\"\n", r.name, r.signal, r.step_deg,
                r.overshoot_pct, r.settling_time_s, r.steady_error_deg, r.max_fin_deg, r.note)
    end
end
println("\nWrote ", normpath(out_path))

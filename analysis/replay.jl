# Offline replay: feed a logged run's commands into the model and compare the
# predicted pitch and yaw with what the IMU measured. CLAUDE_AUV_PJT.md Step 5.
#
# Run from the repo root:
#     julia --project=. analysis/replay.jl logs/<run>.csv
#
# Log columns (written by pi/logger.py), all in MODEL signs and radians:
#     t, thrust_cmd, d1, d2, d3, pitch_meas, yaw_meas, pitch_rate, yaw_rate, roll_meas, mode
#
# What is and is not compared:
#   - θ and ψ are compared (they are measured).
#   - x, z and speed are NOT measured. The replay starts them from a guess
#     (z = 0, speed = model's nominal speed for the first thrust) and they stay
#     model predictions.
#   - Yaw has no sway DOF in the model, so compare ψ only over short windows.
#
# This is OFFLINE replay on a laptop. It is not a real-time on-vehicle model.
using Printf

include(joinpath(@__DIR__, "..", "control", "closed_loop.jl"))

const REPLAY_WINDOW_S = 5.0      # length of the sliding RMS windows

function read_log(path)
    lines = readlines(path)
    header = split(lines[1], ',')
    col(name) = findfirst(==(name), header)
    rows = [split(l, ',') for l in lines[2:end] if !isempty(strip(l))]
    num(name) = [parse(Float64, r[col(name)]) for r in rows]
    return (t = num("t"), thrust = num("thrust_cmd"),
            fins = hcat(num("d1"), num("d2"), num("d3")),
            θ = num("pitch_meas"), ψ = num("yaw_meas"),
            θ_rate = num("pitch_rate"), ψ_rate = num("yaw_rate"))
end

rms(x) = sqrt(sum(abs2, x) / length(x))

"""
    replay(log; p_base = A.p_vals)

Integrates the model with the logged thrust and fin angles held constant
between samples (zero-order hold, the same as on the vehicle). Returns the
predicted θ and ψ at the log's time stamps.
"""
function replay(log; p_base = A.p_vals)
    n = length(log.t)
    p = copy(p_base)
    fin_idx = [A.idx_δ1, A.idx_δ2, A.idx_δ3]

    # Measured: θ, ψ and their rates. Guessed: x = z = 0, ż = 0, and the
    # forward speed (no speed sensor) = nominal speed for the first thrust.
    speed_guess_m_s = nominal_speed(log.thrust[1]; p_base = p_base)
    u = [0.0, 0.0, log.θ[1], log.ψ[1], speed_guess_m_s, 0.0, log.θ_rate[1], log.ψ_rate[1]]

    θ_pred = zeros(n); ψ_pred = zeros(n)
    θ_pred[1] = u[3]; ψ_pred[1] = u[4]
    for k in 1:n-1
        p[A.idx_T] = log.thrust[k]
        p[fin_idx] .= log.fins[k, :]
        u = integrate_model(u, (log.t[k], log.t[k+1]), p)
        θ_pred[k+1] = u[3]; ψ_pred[k+1] = u[4]
    end
    return (; θ_pred, ψ_pred, speed_guess_m_s)
end

function report(log, pred; window_s = REPLAY_WINDOW_S)
    @printf("initial speed guess (model, not measured): %.3f m/s\n", pred.speed_guess_m_s)
    @printf("whole run:  RMS θ error = %.3f°   RMS ψ error = %.3f°\n",
            rad2deg(rms(pred.θ_pred .- log.θ)), rad2deg(rms(pred.ψ_pred .- log.ψ)))
    println("sliding windows:")
    t0 = log.t[1]
    while t0 < log.t[end]
        w = (log.t .>= t0) .& (log.t .< t0 + window_s)
        count(w) > 1 && @printf("  %6.1f – %6.1f s   θ: %.3f°   ψ: %.3f°\n", t0, t0 + window_s,
                                rad2deg(rms(pred.θ_pred[w] .- log.θ[w])),
                                rad2deg(rms(pred.ψ_pred[w] .- log.ψ[w])))
        t0 += window_s
    end
end

function write_comparison(path, log, pred)
    open(path, "w") do io
        println(io, "t,pitch_meas,pitch_pred,yaw_meas,yaw_pred")
        for k in eachindex(log.t)
            @printf(io, "%.3f,%.6f,%.6f,%.6f,%.6f\n", log.t[k], log.θ[k], pred.θ_pred[k],
                    log.ψ[k], pred.ψ_pred[k])
        end
    end
    println("wrote ", normpath(path))
end

if abspath(PROGRAM_FILE) == @__FILE__
    isempty(ARGS) && error("usage: julia --project=. analysis/replay.jl logs/<run>.csv")
    log_path = ARGS[1]
    run_log = read_log(log_path)
    pred = replay(run_log)
    report(run_log, pred)
    write_comparison(replace(log_path, r"\.csv$" => "_replay.csv"), run_log, pred)
end

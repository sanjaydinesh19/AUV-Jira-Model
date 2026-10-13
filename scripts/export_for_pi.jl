# Export small JSON tables so the Raspberry Pi never has to run Symbolics
# (CLAUDE_AUV_PJT.md Section 6, Step 3).
#
# Run from the repo root:  julia --project=. scripts/export_for_pi.jl
#
# Writes into export/:
#   allocation_table.json      pseudo-inverse of the 2×3 fin block vs speed
#   thrust_speed_map.json      thrust -> terminal speed (model prediction)
#   controller_config.json     PID gains and limits (copied from control/config.jl)
#   controller_reference.json  one simulated run, used by the Python tests to
#                              check that the Pi controller matches this one
#
# Everything exported comes from a model with PLACEHOLDER parameters.
using JSON

include(joinpath(@__DIR__, "..", "control", "closed_loop.jl"))

const EXPORT_DIR = joinpath(@__DIR__, "..", "export")
const PLACEHOLDER_NOTE = "Computed from the Julia model with PLACEHOLDER parameters. Not measured."

rows(M) = [collect(r) for r in eachrow(M)]      # matrix -> list of rows for JSON

function write_json(name, data)
    path = joinpath(EXPORT_DIR, name)
    open(io -> JSON.print(io, data, 2), path, "w")
    println("wrote ", normpath(path))
end

mkpath(EXPORT_DIR)

# --- allocation table -------------------------------------------------------
# For each speed: the 3×2 matrix P with  [δ1, δ2, δ3] = P * [pitch_moment, yaw_moment],
# evaluated for a level hull moving straight at that speed.
speeds_m_s = collect(0.20:0.05:1.50)
pinv_tables = [rows(pinv(fin_block(A.B_fun([0.0, 0.0, 0.0, 0.0, s, 0.0, 0.0, 0.0], A.p_vals))))
               for s in speeds_m_s]
write_json("allocation_table.json", Dict(
    "note" => PLACEHOLDER_NOTE,
    "usage" => "fins_rad[3] = pinv[i] (3 rows x 2 cols) * [pitch_moment_Nm, yaw_moment_Nm]; interpolate linearly in speed",
    "signs" => "model convention: theta positive nose DOWN, psi positive counter-clockwise from above",
    "speeds_m_s" => speeds_m_s,
    "pinv" => pinv_tables))

# --- thrust -> speed map ----------------------------------------------------
thrusts_N = [0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 15.0]
write_json("thrust_speed_map.json", Dict(
    "note" => PLACEHOLDER_NOTE * " Terminal forward speed, fins centred, 40 s from rest.",
    "thrust_N" => thrusts_N,
    "speed_m_s" => [nominal_speed(T) for T in thrusts_N]))

# --- gains and limits -------------------------------------------------------
write_json("controller_config.json", Dict(
    "note" => "Copied from control/config.jl. Gains tuned in simulation only; fin limits are placeholders.",
    "control_rate_hz" => CONTROL_RATE_HZ,
    "fin_limit_rad" => FIN_LIMIT_RAD,
    "fin_rate_limit_rad_s" => FIN_RATE_LIMIT_RAD_S,
    "min_control_speed_m_s" => MIN_CONTROL_SPEED_M_S,
    "pitch" => Dict("kp" => PITCH_KP, "ki" => PITCH_KI, "kd" => PITCH_KD,
                    "integral_limit" => PITCH_INTEGRAL_LIMIT),
    "yaw" => Dict("kp" => YAW_KP, "ki" => YAW_KI, "kd" => YAW_KD,
                  "integral_limit" => YAW_INTEGRAL_LIMIT)))

# --- reference run for the Python cross-check -------------------------------
# A pitch step and a heading step with the allocator scheduled on nominal
# speed, exactly as the Pi does it. The Python controller is fed the same
# θ, θ̇, ψ, ψ̇ sequence and must produce the same fin angles.
ref_thrust_N = 8.0
pitch_ref(t) = t < 5.0 ? 0.0 : deg2rad(-5.0)
yaw_ref(t)   = t < 10.0 ? 0.0 : deg2rad(8.0)
ref_log = simulate_closed_loop(thrust_N = ref_thrust_N, t_end_s = 20.0,
                           pitch_ref = pitch_ref, yaw_ref = yaw_ref,
                           speed_source = :nominal)
write_json("controller_reference.json", Dict(
    "note" => "SIMULATED run (placeholder model). Used only to test pi/controller.py against the Julia controller.",
    "thrust_N" => ref_thrust_N,
    "t_s" => ref_log.t,
    "theta_rad" => ref_log.state[:, 3], "psi_rad" => ref_log.state[:, 4],
    "theta_rate_rad_s" => ref_log.state[:, 7], "psi_rate_rad_s" => ref_log.state[:, 8],
    "pitch_ref_rad" => ref_log.pitch_ref, "yaw_ref_rad" => ref_log.yaw_ref,
    "fins_rad" => rows(ref_log.fins)))

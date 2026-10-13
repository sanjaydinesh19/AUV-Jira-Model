# Fin allocator: turns the two moments the PIDs ask for into three fin angles.
#
# The model gives B = ∂Q/∂[T, δ1, δ2, δ3] (4×4, rank 3). The heave and pitch
# rows are proportional in the fin columns, so the fins control only TWO things:
# pitch and yaw. We therefore keep the 2×3 block
#     rows    θ, ψ         (generalized coordinates 3 and 4 of q = [x, z, θ, ψ])
#     columns δ1, δ2, δ3   (actuators 2..4 of [T, δ1, δ2, δ3])
# and solve  Bf * δ = [τθ, τψ]  with the pseudo-inverse: of all fin settings
# that give the wanted moments, it picks the one with the smallest deflections.
using LinearAlgebra

const Q_ROWS_PITCH_YAW = 3:4     # θ, ψ in q = [x, z, θ, ψ]
const ACT_COLS_FINS    = 2:4     # δ1, δ2, δ3 in [T, δ1, δ2, δ3]

fin_block(B) = B[Q_ROWS_PITCH_YAW, ACT_COLS_FINS]

allocate_fins(Bf, pitch_moment_Nm, yaw_moment_Nm) =
    pinv(Bf) * [pitch_moment_Nm, yaw_moment_Nm]

# The ONE place fin angle and rate limits are enforced on the Julia side
# (Section 9). Same logic and same numbers as limit_fins in pi/safety.py.
function limit_fins(fin_cmd_rad, fin_prev_rad, dt_s)
    max_step_rad = FIN_RATE_LIMIT_RAD_S * dt_s
    step  = clamp.(fin_cmd_rad .- fin_prev_rad, -max_step_rad, max_step_rad)
    return clamp.(fin_prev_rad .+ step, -FIN_LIMIT_RAD, FIN_LIMIT_RAD)
end

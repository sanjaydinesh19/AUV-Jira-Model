# Controller numbers: gains, limits and speeds live here and nowhere else
# (CLAUDE_AUV_PJT.md Section 10). Radians and SI units throughout.
#
# The gains were tuned in simulation on a model whose parameters are all
# PLACEHOLDERS. Expect to retune every one of them on the real vehicle.

const CONTROL_RATE_HZ = 20.0                 # control loop rate
const CONTROL_DT_S    = 1.0 / CONTROL_RATE_HZ

# Fin limits. Must match pi/config.py.
const FIN_LIMIT_RAD        = deg2rad(20.0)   # PLACEHOLDER: ask for the real servo limits
const FIN_RATE_LIMIT_RAD_S = deg2rad(60.0)   # PLACEHOLDER: not specified anywhere yet

# Below this forward speed the fins have almost no authority (force ∝ speed²),
# so the allocator commands neutral fins instead of saturating them.
const MIN_CONTROL_SPEED_M_S = 0.2

# Pitch hold. Output is a pitch moment [N·m] (generalized force on θ).
const PITCH_KP = 6.0            # N·m per rad
const PITCH_KI = 3.0            # N·m per rad·s
const PITCH_KD = 3.0            # N·m per rad/s
const PITCH_INTEGRAL_LIMIT = 0.5   # rad·s, anti-windup clamp on ∫error

# Heading hold. Output is a yaw moment [N·m] (generalized force on ψ).
const YAW_KP = 6.0
const YAW_KI = 3.0
const YAW_KD = 3.0
const YAW_INTEGRAL_LIMIT = 0.5

# Solver settings: the same as the notebook.
const SIM_RELTOL = 1e-6
const SIM_ABSTOL = 1e-6

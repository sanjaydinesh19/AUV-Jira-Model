"""All Pi-side numbers live here (CLAUDE.md Section 10: no magic numbers).

Radians everywhere. Anything marked PLACEHOLDER is not measured and must be
replaced with a real value before an in-water run.
"""
import math

CONTROL_RATE_HZ = 20.0  # control loop rate (Section 6, Step 1)

# Fail-safe thresholds (Section 9)
IMU_TIMEOUT_S = 0.5        # no IMU data for longer than this -> thruster off, fins neutral
PITCH_ABORT_RAD = 0.6      # |pitch| above this -> thruster off, fins neutral

# Fin limits
FIN_LIMIT_RAD = math.radians(20.0)        # PLACEHOLDER: ask user for real servo limits
FIN_RATE_LIMIT_RAD_S = math.radians(60.0)  # PLACEHOLDER: not specified anywhere yet

# CSV log columns, in the order fixed by Section 6, Step 4
LOG_COLUMNS = [
    "t", "thrust_cmd", "d1", "d2", "d3",
    "pitch_meas", "yaw_meas", "pitch_rate", "yaw_rate", "roll_meas", "mode",
]

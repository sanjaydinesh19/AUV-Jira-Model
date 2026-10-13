"""Simulated IMU source so the Pi code runs on a laptop with no hardware.

IMPORTANT: this is a SCRIPTED motion profile (like tilting the hull by hand on
the bench). It is NOT the AUV dynamics model and its output is not a result.
It exists only to exercise the sign conversion, the fail-safe and the logger.

Output is in the MAVLink convention (pitch + nose up, yaw + clockwise), exactly
as a real Pixhawk would send it, so the same conversion code is used for both.
"""
import math

from imu_convert import MavAttitude, wrap_pi

# Scripted profile (bench-test script, not physics)
PITCH_AMP_RAD = 0.15         # hand tilt amplitude
PITCH_FREQ_HZ = 0.25         # hand tilt frequency
YAW_START_RAD = 3.0          # start close to +pi so the heading wraps during the run
YAW_RATE_RAD_S = 0.05        # slow clockwise turn
DROPOUT_START_S = 5.0        # IMU goes silent here ...
DROPOUT_END_S = 5.8          # ... until here (longer than the 0.5 s timeout)
OVERPITCH_START_S = 8.0      # after this the nose is pushed steadily up
OVERPITCH_RATE_RAD_S = 0.35  # so that |pitch| crosses the 0.6 rad abort limit


class SimulatedImu:
    """Returns a MavAttitude for time t, or None during the scripted dropout."""

    def read(self, t_s):
        if DROPOUT_START_S <= t_s < DROPOUT_END_S:
            return None

        w = 2.0 * math.pi * PITCH_FREQ_HZ
        if t_s < OVERPITCH_START_S:
            pitch = PITCH_AMP_RAD * math.sin(w * t_s)
            pitchspeed = PITCH_AMP_RAD * w * math.cos(w * t_s)
        else:
            # sin(w * 8 s) = 0, so the ramp starts from level
            pitch = OVERPITCH_RATE_RAD_S * (t_s - OVERPITCH_START_S)
            pitchspeed = OVERPITCH_RATE_RAD_S

        return MavAttitude(
            t_s=t_s,
            roll_rad=0.0,
            pitch_rad=pitch,
            yaw_rad=wrap_pi(YAW_START_RAD + YAW_RATE_RAD_S * t_s),
            rollspeed_rad_s=0.0,
            pitchspeed_rad_s=pitchspeed,
            yawspeed_rad_s=YAW_RATE_RAD_S,
        )

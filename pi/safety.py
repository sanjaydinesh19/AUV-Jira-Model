"""Fin limits and fail-safe checks. CLAUDE.md Section 9.

limit_fins is the ONE function that enforces fin angle and rate limits; the
simulation and the hardware must both call it.
"""
import config

MODE_OK = "OK"
MODE_IMU_TIMEOUT = "FAILSAFE_IMU_TIMEOUT"
MODE_PITCH_ABORT = "FAILSAFE_PITCH"


def limit_fins(fin_cmd_rad, fin_prev_rad, dt_s):
    """Rate-limit, then saturate, each fin command. Returns a new list."""
    max_step_rad = config.FIN_RATE_LIMIT_RAD_S * dt_s
    limited = []
    for cmd, prev in zip(fin_cmd_rad, fin_prev_rad):
        step = max(-max_step_rad, min(max_step_rad, cmd - prev))
        angle = prev + step
        limited.append(max(-config.FIN_LIMIT_RAD, min(config.FIN_LIMIT_RAD, angle)))
    return limited


def check_failsafe(t_now_s, t_last_imu_s, theta_rad):
    """Return MODE_OK, or the reason the thruster must be off and fins neutral.

    theta_rad is the last valid pitch; it may be None before the first sample.
    """
    if t_now_s - t_last_imu_s > config.IMU_TIMEOUT_S:
        return MODE_IMU_TIMEOUT
    if theta_rad is not None and abs(theta_rad) > config.PITCH_ABORT_RAD:
        return MODE_PITCH_ABORT
    return MODE_OK

"""Pixhawk (MAVLink ATTITUDE) -> model sign conversion. CLAUDE.md Section 7.

This is the ONE place where the conversion happens. MAVLink attitude is NED/FRD:
pitch positive nose UP, yaw positive clockwise seen from above. The Julia model
is the opposite on both (theta positive nose DOWN, psi positive counter-clockwise).

Valid for small angles (|theta| < ~0.3 rad), where body rates ~ Euler rates.
Assumes the Pixhawk axes align with the hull (x forward, y right, z down).
"""
import math
from dataclasses import dataclass


@dataclass
class MavAttitude:
    """One MAVLink ATTITUDE sample, untouched (NED/FRD, radians, rad/s)."""
    t_s: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    rollspeed_rad_s: float
    pitchspeed_rad_s: float
    yawspeed_rad_s: float


@dataclass
class ModelAttitude:
    """The same sample in the model's convention (q = [x, z, theta, psi])."""
    t_s: float
    theta_rad: float       # pitch, positive nose DOWN
    theta_rate_rad_s: float
    psi_rad: float         # heading relative to start, positive counter-clockwise
    psi_rate_rad_s: float
    roll_rad: float        # logged only: the 4-DOF model has no roll


def wrap_pi(angle_rad):
    """Wrap an angle to (-pi, pi]."""
    wrapped = math.fmod(angle_rad + math.pi, 2.0 * math.pi)
    if wrapped <= 0.0:
        wrapped += 2.0 * math.pi
    return wrapped - math.pi


def mav_to_model(att, yaw_mav_at_start_rad):
    """Convert one MAVLink ATTITUDE sample to model theta/psi and their rates.

    Yaw is taken relative to the heading at start, because the model's psi
    starts at 0 while the compass heading is measured from north.
    """
    return ModelAttitude(
        t_s=att.t_s,
        theta_rad=-att.pitch_rad,
        theta_rate_rad_s=-att.pitchspeed_rad_s,
        psi_rad=-wrap_pi(att.yaw_rad - yaw_mav_at_start_rad),
        psi_rate_rad_s=-att.yawspeed_rad_s,
        roll_rad=att.roll_rad,
    )

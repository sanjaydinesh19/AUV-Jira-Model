"""Pitch hold + heading hold for the Pi. CLAUDE.md Section 6, Step 3.

A Python copy of the Julia controller in control/ (pid.jl, allocator.jl,
closed_loop.jl). test_controller.py checks that both give the same fin angles.

The Pi never runs the model. It reads three JSON files written by
scripts/export_for_pi.jl:
    controller_config.json   PID gains
    thrust_speed_map.json    thrust -> nominal speed (model prediction)
    allocation_table.json    moments -> fin angles, as a function of speed

There is NO speed sensor, so the allocator is scheduled on the NOMINAL speed
for the commanded thrust. All three files come from a model with placeholder
parameters; the gains will need retuning on the real vehicle.
"""
import json
import os

import config
from imu_convert import wrap_pi
from safety import limit_fins

EXPORT_DIR = os.path.join(os.path.dirname(__file__), "..", "export")


def load_export(name):
    with open(os.path.join(EXPORT_DIR, name), encoding="utf-8") as f:
        return json.load(f)


def interp(x, xs, ys):
    """Linear interpolation, clamped at both ends. xs must be increasing."""
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            w = (x - xs[i - 1]) / (xs[i] - xs[i - 1])
            return ys[i - 1] + w * (ys[i] - ys[i - 1])


class PID:
    """D term on the measured rate (gyro); integral clamped and freezable."""

    def __init__(self, kp, ki, kd, integral_limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit = integral_limit
        self.integral = 0.0

    def update(self, error, rate, dt_s, integrate=True):
        if integrate:
            self.integral += error * dt_s
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        return self.kp * error + self.ki * self.integral - self.kd * rate


class Controller:
    def __init__(self, thrust_N):
        cfg = load_export("controller_config.json")
        speed_map = load_export("thrust_speed_map.json")
        table = load_export("allocation_table.json")

        self.dt_s = 1.0 / config.CONTROL_RATE_HZ
        self.pitch_pid = PID(**cfg["pitch"])
        self.yaw_pid = PID(**cfg["yaw"])
        self.fins_rad = [0.0, 0.0, 0.0]
        self.saturated = False

        # Nominal speed for this thrust, then the allocation matrix at that speed.
        self.speed_m_s = interp(thrust_N, speed_map["thrust_N"], speed_map["speed_m_s"])
        self.low_speed = self.speed_m_s < cfg["min_control_speed_m_s"]
        if self.low_speed:
            print(f"WARNING: nominal speed {self.speed_m_s:.2f} m/s is below the minimum "
                  f"control speed; fins stay neutral.")
        # 3 rows (fins) x 2 columns (pitch moment, yaw moment)
        self.pinv = [[interp(self.speed_m_s, table["speeds_m_s"],
                             [m[row][col] for m in table["pinv"]])
                      for col in range(2)] for row in range(3)]

    def step(self, att, pitch_ref_rad, yaw_ref_rad):
        """One 20 Hz step. `att` is a ModelAttitude (model signs). Returns 3 fin angles [rad]."""
        integrate = not self.saturated and not self.low_speed
        pitch_moment_Nm = self.pitch_pid.update(
            pitch_ref_rad - att.theta_rad, att.theta_rate_rad_s, self.dt_s, integrate)
        yaw_moment_Nm = self.yaw_pid.update(
            wrap_pi(yaw_ref_rad - att.psi_rad), att.psi_rate_rad_s, self.dt_s, integrate)

        if self.low_speed:
            fin_cmd_rad = [0.0, 0.0, 0.0]
        else:
            fin_cmd_rad = [row[0] * pitch_moment_Nm + row[1] * yaw_moment_Nm
                           for row in self.pinv]
        self.saturated = any(abs(c) > config.FIN_LIMIT_RAD for c in fin_cmd_rad)
        self.fins_rad = limit_fins(fin_cmd_rad, self.fins_rad, self.dt_s)
        return self.fins_rad

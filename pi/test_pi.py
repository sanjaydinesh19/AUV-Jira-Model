"""Unit tests for the Pi-side code. Run:  python -m unittest discover -s pi -v"""
import math
import unittest

import config
from imu_convert import MavAttitude, mav_to_model, wrap_pi
from safety import (MODE_IMU_TIMEOUT, MODE_OK, MODE_PITCH_ABORT,
                    check_failsafe, limit_fins)


def mav(pitch=0.0, yaw=0.0, pitchspeed=0.0, yawspeed=0.0, roll=0.0):
    return MavAttitude(0.0, roll, pitch, yaw, 0.0, pitchspeed, yawspeed)


class TestSignConversion(unittest.TestCase):
    """CLAUDE.md Section 7 table."""

    def test_nose_up_is_negative_theta(self):
        out = mav_to_model(mav(pitch=0.1, pitchspeed=0.2), 0.0)
        self.assertAlmostEqual(out.theta_rad, -0.1)
        self.assertAlmostEqual(out.theta_rate_rad_s, -0.2)

    def test_clockwise_turn_is_negative_psi(self):
        out = mav_to_model(mav(yaw=0.3, yawspeed=0.05), 0.0)
        self.assertAlmostEqual(out.psi_rad, -0.3)
        self.assertAlmostEqual(out.psi_rate_rad_s, -0.05)

    def test_yaw_is_relative_to_start(self):
        out = mav_to_model(mav(yaw=1.2), 1.0)
        self.assertAlmostEqual(out.psi_rad, -0.2)

    def test_yaw_wraps_across_pi(self):
        # Start at +3.0 rad, turn 0.4 rad clockwise: compass reads 3.4 - 2*pi.
        out = mav_to_model(mav(yaw=3.4 - 2.0 * math.pi), 3.0)
        self.assertAlmostEqual(out.psi_rad, -0.4)

    def test_roll_passes_through(self):
        self.assertAlmostEqual(mav_to_model(mav(roll=0.07), 0.0).roll_rad, 0.07)

    def test_wrap_pi_range(self):
        self.assertAlmostEqual(wrap_pi(math.pi), math.pi)    # +pi is included
        self.assertAlmostEqual(wrap_pi(-math.pi), math.pi)   # -pi maps to +pi
        self.assertAlmostEqual(wrap_pi(2.0 * math.pi + 0.1), 0.1)
        self.assertAlmostEqual(wrap_pi(-0.1), -0.1)


class TestSafety(unittest.TestCase):
    """CLAUDE.md Section 9."""

    def test_fin_saturation(self):
        big = 2.0 * config.FIN_LIMIT_RAD
        out = limit_fins([big, -big, 0.0], [big, -big, 0.0], dt_s=0.05)
        self.assertAlmostEqual(out[0], config.FIN_LIMIT_RAD)
        self.assertAlmostEqual(out[1], -config.FIN_LIMIT_RAD)
        self.assertAlmostEqual(out[2], 0.0)

    def test_fin_rate_limit(self):
        dt_s = 0.05
        out = limit_fins([config.FIN_LIMIT_RAD] * 3, [0.0] * 3, dt_s)
        for angle in out:
            self.assertAlmostEqual(angle, config.FIN_RATE_LIMIT_RAD_S * dt_s)

    def test_failsafe_ok(self):
        self.assertEqual(check_failsafe(1.0, 0.9, 0.1), MODE_OK)

    def test_failsafe_imu_timeout(self):
        self.assertEqual(check_failsafe(1.0, 0.4, 0.1), MODE_IMU_TIMEOUT)

    def test_failsafe_pitch_either_sign(self):
        self.assertEqual(check_failsafe(1.0, 1.0, 0.61), MODE_PITCH_ABORT)
        self.assertEqual(check_failsafe(1.0, 1.0, -0.61), MODE_PITCH_ABORT)


if __name__ == "__main__":
    unittest.main()

"""Cross-check: the Pi controller must reproduce the Julia controller.

export/controller_reference.json holds one SIMULATED closed-loop run from Julia
(theta, psi, rates in, fin angles out). We feed the same inputs to
controller.Controller and compare the fin angles.

The small tolerance covers the one intended difference: the Pi interpolates the
allocation table in speed, Julia evaluates the model at the exact speed.

Run:  python -m unittest discover -s pi -v
"""
import math
import unittest

import config
from controller import Controller, interp, load_export
from imu_convert import ModelAttitude

FIN_TOLERANCE_RAD = math.radians(0.1)


class TestControllerMatchesJulia(unittest.TestCase):
    def test_limits_match_julia_config(self):
        cfg = load_export("controller_config.json")
        self.assertAlmostEqual(cfg["fin_limit_rad"], config.FIN_LIMIT_RAD)
        self.assertAlmostEqual(cfg["fin_rate_limit_rad_s"], config.FIN_RATE_LIMIT_RAD_S)
        self.assertAlmostEqual(cfg["control_rate_hz"], config.CONTROL_RATE_HZ)

    def test_fin_commands_match_reference_run(self):
        ref = load_export("controller_reference.json")
        ctrl = Controller(ref["thrust_N"])
        worst_rad = 0.0
        for k, t_s in enumerate(ref["t_s"]):
            att = ModelAttitude(t_s, ref["theta_rad"][k], ref["theta_rate_rad_s"][k],
                                ref["psi_rad"][k], ref["psi_rate_rad_s"][k], 0.0)
            fins = ctrl.step(att, ref["pitch_ref_rad"][k], ref["yaw_ref_rad"][k])
            for got, want in zip(fins, ref["fins_rad"][k]):
                worst_rad = max(worst_rad, abs(got - want))
        print(f"\n  worst fin difference vs Julia: {math.degrees(worst_rad):.4f} deg "
              f"over {len(ref['t_s'])} steps")
        self.assertLess(worst_rad, FIN_TOLERANCE_RAD)

    def test_low_thrust_gives_neutral_fins(self):
        ctrl = Controller(thrust_N=0.2)
        self.assertTrue(ctrl.low_speed)
        att = ModelAttitude(0.0, 0.2, 0.0, 0.1, 0.0, 0.0)
        self.assertEqual(ctrl.step(att, 0.0, 0.0), [0.0, 0.0, 0.0])

    def test_interp_clamps_and_interpolates(self):
        self.assertEqual(interp(-1.0, [0.0, 1.0], [10.0, 20.0]), 10.0)
        self.assertEqual(interp(5.0, [0.0, 1.0], [10.0, 20.0]), 20.0)
        self.assertAlmostEqual(interp(0.25, [0.0, 1.0], [10.0, 20.0]), 12.5)


if __name__ == "__main__":
    unittest.main()

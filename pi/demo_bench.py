"""Laptop bench demo: simulated IMU -> sign conversion -> fail-safe -> CSV log.

Run:  python pi/demo_bench.py

There is NO controller and NO vehicle model here. Thrust is 0 and the fins stay
neutral, because the pitch/heading controller needs the allocation table that
comes from the Julia model (Step 0 / Step 3), which is not in the repo yet.
The IMU data is a scripted profile (see imu_sim.py), not a measurement.
"""
import math
import os

import config
from imu_convert import mav_to_model
from imu_sim import SimulatedImu
from logger import RunLogger
from safety import MODE_OK, check_failsafe

RUN_TIME_S = 10.0
PRINT_EVERY_S = 0.5
LOG_PATH = os.path.join(os.path.dirname(__file__), "..", "logs", "bench_sim_demo.csv")


def main():
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
    imu = SimulatedImu()
    log = RunLogger(LOG_PATH)
    dt_s = 1.0 / config.CONTROL_RATE_HZ

    yaw_mav_at_start_rad = None
    t_last_imu_s = 0.0
    last = None  # last valid ModelAttitude
    prev_mode = None

    print("SIMULATED IMU (scripted bench profile) - not a measurement, not the model")
    print(f"{'t [s]':>6} {'pitch_mav':>10} {'yaw_mav':>9} | {'theta':>8} {'psi':>8}  [deg]  mode")

    n_steps = int(round(RUN_TIME_S / dt_s))
    for k in range(n_steps + 1):
        t_s = k * dt_s
        mav = imu.read(t_s)
        if mav is not None:
            if yaw_mav_at_start_rad is None:
                yaw_mav_at_start_rad = mav.yaw_rad
            last = mav_to_model(mav, yaw_mav_at_start_rad)
            t_last_imu_s = t_s

        mode = check_failsafe(t_s, t_last_imu_s, last.theta_rad if last else None)

        # No controller yet: thruster off, fins neutral in every mode.
        log.write(
            t=f"{t_s:.3f}", thrust_cmd=0.0, d1=0.0, d2=0.0, d3=0.0,
            pitch_meas=f"{last.theta_rad:.6f}", yaw_meas=f"{last.psi_rad:.6f}",
            pitch_rate=f"{last.theta_rate_rad_s:.6f}", yaw_rate=f"{last.psi_rate_rad_s:.6f}",
            roll_meas=f"{last.roll_rad:.6f}", mode=mode,
        )

        if mode != prev_mode and prev_mode is not None:
            print(f"{t_s:6.2f}  >>> mode change: {prev_mode} -> {mode}")
        prev_mode = mode

        if k % int(round(PRINT_EVERY_S / dt_s)) == 0:
            if mav is None:
                raw = f"{'--':>10} {'--':>9}"
            else:
                raw = f"{math.degrees(mav.pitch_rad):10.2f} {math.degrees(mav.yaw_rad):9.2f}"
            print(f"{t_s:6.2f} {raw} | {math.degrees(last.theta_rad):8.2f} "
                  f"{math.degrees(last.psi_rad):8.2f}         {mode}")

    log.close()
    print(f"\nLogged {n_steps + 1} rows to {os.path.normpath(LOG_PATH)}")
    if mode != MODE_OK:
        print(f"Run ended in {mode} (scripted, expected).")


if __name__ == "__main__":
    main()

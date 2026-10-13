"""Laptop bench demo: simulated IMU -> sign conversion -> controller -> fail-safe -> CSV log.

Run:  python pi/demo_bench.py

This is the software version of the in-air bench sign check (CLAUDE.md
Section 6, Step 4.1): the hull is "tilted by hand" and we look at which way
the fins are commanded. Nothing here is a closed loop: the scripted IMU does
not react to the fins, and there is no vehicle model on the Pi.

  - The IMU data is a scripted profile (see imu_sim.py), not a measurement.
  - The thruster command stays 0 (bench, in air). The controller still needs a
    speed to scale the fin commands, so it is scheduled for
    BENCH_SCHEDULE_THRUST_N as if the vehicle were cruising.
  - In any fail-safe mode the fins are commanded neutral.
"""
import math
import os

import config
from controller import Controller
from imu_convert import mav_to_model
from imu_sim import SimulatedImu
from logger import RunLogger
from safety import MODE_OK, check_failsafe

RUN_TIME_S = 10.0
PRINT_EVERY_S = 0.5
BENCH_SCHEDULE_THRUST_N = 8.0   # only selects the allocation table row; thruster stays off
PITCH_REF_RAD = 0.0
YAW_REF_RAD = 0.0
LOG_PATH = os.path.join(os.path.dirname(__file__), "..", "logs", "bench_sim_demo.csv")


def main():
    os.makedirs(os.path.dirname(LOG_PATH), exist_ok=True)
    imu = SimulatedImu()
    ctrl = Controller(BENCH_SCHEDULE_THRUST_N)
    log = RunLogger(LOG_PATH)
    dt_s = 1.0 / config.CONTROL_RATE_HZ

    yaw_mav_at_start_rad = None
    t_last_imu_s = 0.0
    last = None  # last valid ModelAttitude
    prev_mode = None
    fins_rad = [0.0, 0.0, 0.0]

    print("SIMULATED IMU (scripted bench profile) - not a measurement, not the model")
    print(f"{'t [s]':>6} {'pitch_mav':>10} {'yaw_mav':>9} | {'theta':>7} {'psi':>7} | "
          f"{'d1':>6} {'d2':>6} {'d3':>6}  [deg]  mode")

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

        if mode == MODE_OK and mav is not None:
            fins_rad = ctrl.step(last, PITCH_REF_RAD, YAW_REF_RAD)
        elif mode != MODE_OK:
            fins_rad = [0.0, 0.0, 0.0]      # fail-safe: fins neutral (thruster is already off)
            ctrl.fins_rad = fins_rad
        # else: one missed IMU sample inside the timeout -> hold the last fin command

        log.write(
            t=f"{t_s:.3f}", thrust_cmd=0.0,
            d1=f"{fins_rad[0]:.6f}", d2=f"{fins_rad[1]:.6f}", d3=f"{fins_rad[2]:.6f}",
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
            fins = " ".join(f"{math.degrees(d):6.1f}" for d in fins_rad)
            print(f"{t_s:6.2f} {raw} | {math.degrees(last.theta_rad):7.2f} "
                  f"{math.degrees(last.psi_rad):7.2f} | {fins}         {mode}")

    log.close()
    print(f"\nLogged {n_steps + 1} rows to {os.path.normpath(LOG_PATH)}")
    if mode != MODE_OK:
        print(f"Run ended in {mode} (scripted, expected).")


if __name__ == "__main__":
    main()

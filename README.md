# AUV pitch and heading hold (4-DOF Julia model + Pixhawk IMU)

Capstone project, VIT Chennai: *Real-Time Nonlinear Dynamics Modeling of a 4-DOF
Autonomous Underwater Vehicle*. Torpedo hull, one rear thruster, three servo
fins (Y-tail), Pixhawk IMU, Raspberry Pi. The full brief is in
[CLAUDE_AUV_PJT.md](CLAUDE_AUV_PJT.md).

> **Read this first.** Every mass, geometry and hydrodynamic number in the model
> is a **placeholder**, and **no real vehicle data exists in this repo yet**.
> All results below are simulation results. Depth and speed are model
> predictions, never measurements (there is no depth or speed sensor). Heading
> is an approximation because the model has no sway degree of freedom.

## Status

| Step | What | State |
|---|---|---|
| 0 | Model extracted from the notebook, golden tests | **Done.** 41/41 tests pass |
| 1 | Closed-loop simulation: pitch hold, heading hold, fin allocator | **Done in simulation.** 23/23 tests pass |
| 2 | Pixhawk IMU interface | Sign conversion and simulated IMU done and unit-tested. **Real MAVLink reader not written** (firmware, port and baud rate not known yet) |
| 3 | Controller on the Pi | Controller and JSON export done; the Python controller matches the Julia one to 0.05°. **Actuator driver not written** (who drives the servos and ESC is not known yet) |
| 4 | Pool tests and logging | **Not started.** Needs hardware. Logger and CSV format are ready |
| 5 | Offline replay and tuning | Replay script written and self-tested on synthetic logs. **No real log to replay yet** |
| 6 | Deliverables | Simulation metrics only so far, see [docs/results.md](docs/results.md) |

## Reproduce everything

Needs Julia (tested on **1.13.1**) and Python 3 (tested on 3.12, standard library only).

```
julia --project=. -e "using Pkg; Pkg.instantiate()"    # once, takes several minutes

julia --project=. test/golden.jl                       # Step 0: model vs notebook numbers
julia --project=. control/run_closed_loop_tests.jl     # Step 1: closed-loop scenarios + metrics
julia --project=. scripts/export_for_pi.jl             # Step 3: JSON tables for the Pi
julia --project=. test/replay_selftest.jl              # Step 5: replay plumbing check

python -m unittest discover -s pi -v                   # Pi-side unit tests
python pi/demo_bench.py                                # bench sign-check demo on a laptop
```

When a real log exists: `julia --project=. analysis/replay.jl logs/<run>.csv`.

## Layout

```
notebooks/AUV_4DOF_PJT_CODE_JULIA.pdf   original notebook (PDF export), untouched
src/auv_model.jl                        the notebook's model cells, in dependency order
test/golden.jl                          regression tests: the notebook's own outputs
test/replay_selftest.jl                 replay check on synthetic logs
control/                                config (gains, limits), PID, allocator, closed-loop sim
scripts/export_for_pi.jl                writes export/*.json
export/                                 allocation table, thrust-speed map, gains, reference run
pi/                                     IMU conversion, simulated IMU, controller, fail-safe, logger
analysis/replay.jl                      offline replay of a logged run through the model
analysis/closed_loop_metrics.csv        output of the closed-loop scenarios
logs/                                   CSV runs (only a scripted bench demo so far)
docs/results.md                         results and how to read them
```

## About the notebook and the Julia version

The notebook was supplied as a **PDF export with no text layer**, so
`src/auv_model.jl` was transcribed from the page images. The proof that the
transcription is right is `test/golden.jl`: it reproduces every number the
notebook printed (baseline run, thrust sweep, fin steps, allocation matrix,
class table, switched ablation).

The PDF does not record which Julia version the notebook ran on. What the code
shows: it is a **Pluto** notebook, and it uses `substitute(...; fold = Val(true))`,
which is Symbolics 7 syntax, plus `AutoFiniteDiff()` and `build_function(...; cse = true)`.
So it was written against a recent stack (Symbolics 7, Julia 1.10 or newer).
This repo runs it on Julia 1.13.1 with Symbolics 7.45 and OrdinaryDiffEq 7.8;
exact versions are locked in `Manifest.toml`.

Differences from the notebook (none change the physics):

- Plotting cells (WGLMakie) and the SymPy helpers are left out.
- `DifferentialEquations` is replaced by `OrdinaryDiffEq` (same solvers, much smaller install).
- Cells that only run a simulation or print a check moved into `test/golden.jl`.

## Conventions (from the notebook, do not change)

- `q = [x, z, θ, ψ]`, state `u = [x, z, θ, ψ, ẋ, ż, θ̇, ψ̇]`. World z is **up**.
- **θ positive = nose down. ψ positive = counter-clockwise seen from above.**
- The Pixhawk (MAVLink, NED) is the opposite on both. The conversion is in one
  function, `pi/imu_convert.py: mav_to_model`, with unit tests.
- Fin 1 is the top fin, fins 2 and 3 the lower pair. `δ2 = −δ3 = +d` pitches the
  nose up; `δ1 > 0` alone turns ψ negative.
- Radians everywhere inside the code. Degrees only when printing.

## Placeholders (replace before trusting any number)

- **Model:** mass 14 kg, CG and CB offsets, buoyancy 139 N, inertia, thruster and
  fin positions, all drag and added-mass coefficients, fin area and lift slope,
  and water density 1025 kg/m³ (seawater; a pool is about 998).
- **Controller:** fin limit ±20° and fin rate limit 60°/s are guesses; the PID
  gains were tuned only against the placeholder model.

## Known issues (stated honestly)

1. The check "Σ h_classes == model.h" fails as a *symbolic* identity in the
   notebook. It holds *numerically* to round-off (about 1e-15), which is what
   `test/golden.jl` tests. It is **numerically verified, not a symbolic proof**.
2. **No sway DOF.** The hull yaws but its origin moves along world x only, so
   the model pulls ψ back toward 0. Yaw results are an approximation; compare
   yaw only over short windows.
3. Yaw traces at the 1e-16 level in symmetric runs are round-off, not physics.
4. Changing the CG through the parameter vector (`c_x1`, `c_z1`) changes only
   the inertial terms: the gravity force is applied at the fixed `com_val` in
   the notebook. A real CG change means editing `com_val` and rebuilding.
5. The fail-safe in `pi/safety.py` does not latch: it clears itself when the
   IMU returns. Decide before pool tests whether it should stay tripped until
   re-armed.

## Still needed from the team

1. Pixhawk firmware (ArduSub / ArduPilot / PX4), port and baud rate to the Pi.
2. Who drives the servos and the ESC: Pixhawk outputs, an ESP32, or the Pi.
3. Servo angle limits and the positive direction of each fin.
4. Real mass, CG, CB and inertia from the SolidWorks mass-properties export.
5. Fresh water or seawater; is the ESC bidirectional.

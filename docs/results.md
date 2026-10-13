# Results so far

**Everything on this page is simulation on a model with placeholder parameters.**
There is no pool data yet, so there is no predicted-vs-measured comparison and
no table of tuned parameters. Those sections are empty on purpose.

## Step 0: the extracted model reproduces the notebook

`julia --project=. test/golden.jl` gives 41/41 passes.

| Check | Notebook | This repo |
|---|---|---|
| Baseline 30 s, T = 8 N: x | 25.01 m | 25.0107 m |
| z | −0.3997 m | −0.39970 m |
| θ | 0.09109 rad | 0.091088 rad |
| ẋ | 0.8289 m/s | 0.82887 m/s |
| ż | −0.01381 m/s | −0.013809 m/s |
| Thrust sweep, terminal ẋ at T = 2, 5, 10, 15 N | 0.362, 0.633, 0.939, 1.173 m/s | 0.362, 0.633, 0.939, 1.173 m/s |
| Fin step δ2 = −δ3 = 0.2: θ | ≈ −0.14 rad | −0.1368 rad |
| Fin step δ1 = 0.2: ψ | ≈ −0.10 rad | −0.1015 rad |
| rank(B) at u0 | 3 | 3 |
| Σ parts − h (numerical) | ≤ ~1e-15 | ≤ 1.8e-15 |
| `dynamics_sw!` vs `compile_ode` | ~1e-15 | ≤ 4.2e-16 |
| Pitch trim, `dxh^2` class OFF | 0.0911 → ≈ 0.032 | 0.09107 → 0.03221 (64.6% drop) |

The last row is the switched-ablation result: the `ẋ²` class (the Munk moment
from the added-mass difference) is what sets most of the pitch trim.

## Step 1: closed-loop simulation

`julia --project=. control/run_closed_loop_tests.jl` gives 23/23 passes and writes
`analysis/closed_loop_metrics.csv`. Controller at 20 Hz, fins limited to ±20°
and 60°/s (both placeholders). Each run holds θ = ψ = 0 for 15 s, then steps.
Settling band is ±5% of the step.

| Scenario | Step | Overshoot | Settling time | Steady error | Max fin |
|---|---|---|---|---|---|
| Cancel the natural trim, T = 8 N | −5.2° | 10.4% | 4.60 s | 0.000° | 10.6° |
| Pitch step −5° (nose up), T = 8 N | −5.0° | 10.9% | 4.60 s | 0.000° | 14.7° |
| Pitch step +5° (nose down), T = 8 N | +5.0° | 10.0% | 4.55 s | 0.000° | 5.0° |
| Heading step +10°, T = 8 N | +10.0° | 0.0% | 4.30 s | 0.000° | 18.7° |
| Pitch step −5°, T = 5 N (slower) | −5.0° | 0.0% | 4.90 s | 0.000° | 20.0° (saturates) |
| Pitch step −5°, T = 12 N (faster) | −5.0° | 15.3% | 4.95 s | 0.000° | 9.8° |
| Pitch step −5°, allocator on nominal speed | −5.0° | 11.7% | 4.30 s | 0.000° | 14.8° |
| Saturation: ask −25°, then back to 0 | return step 19.3° | 18.8% | 3.50 s | 0.000° | 20.0° |

Disturbance (buoyancy changed by ±2% at t = 15 s while holding θ = 0): peak
pitch error 2.2° (+2%) and 2.3° (−2%), returning to 0.000° steady error.

Low speed, from rest:

- T = 0.5 N: the hull only reaches 0.135 m/s, below the 0.2 m/s minimum control
  speed, so the fins stay neutral for the whole run. Pitch stays at +1.1°. No blow-up.
- T = 2 N: fins neutral for the first 1.9 s, then the controller takes over and
  holds θ = 0 at 0.37 m/s. The fins hit the ±20° limit on the way.

How to read these:

- **Steady error is zero because of the integrator**, which is also what cancels
  the natural nose-down trim (θ ≈ +0.091 rad with fins centred).
- **Nose up costs more fin than nose down** (14.7° against 5.0° for the same 5°)
  because nose up works against that natural trim.
- **Authority falls with speed².** At T = 5 N the same step already saturates the fins.
- **The −25° request cannot be met.** The hull reaches about −19.3° with fins on
  the limit; the integral is frozen while saturated, so the return to 0 is clean.
- **The heading numbers are optimistic.** With no sway DOF the model resists yaw
  on its own. On the real vehicle expect different behaviour.

## Step 3: the Pi controller matches the Julia controller

`python -m unittest discover -s pi -v` gives 15/15 passes. One simulated 20 s run
(pitch step and heading step) is replayed through `pi/controller.py`; the worst
fin-angle difference from the Julia controller is **0.049°** over 400 steps.
The difference comes from the Pi interpolating the allocation table in speed.

Thrust to nominal speed map used by the Pi (model prediction, not measured):

| Thrust [N] | 0.5 | 1 | 2 | 3 | 4 | 5 | 6 | 8 | 10 | 12 | 15 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| Speed [m/s] | 0.135 | 0.227 | 0.362 | 0.467 | 0.555 | 0.633 | 0.704 | 0.829 | 0.939 | 1.038 | 1.173 |

## Step 5: replay self-test

`julia --project=. test/replay_selftest.jl` gives 3/3 passes, on **synthetic** logs:

- Same parameters for the simulated "vehicle" and the model: RMS θ error 0.00000°.
- "Vehicle" simulated with a 20% lower fin lift slope `CLa`: RMS θ error 2.7°.

This shows the replay code is wired correctly and can see a wrong parameter.
It says nothing about how well the model matches the real vehicle.

## Not available yet

- Predicted vs measured θ and ψ from a real run, with RMS error.
- Table of parameters changed during tuning.
- Bench verification of the IMU sign conversion on the real Pixhawk.

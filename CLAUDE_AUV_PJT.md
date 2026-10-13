# CLAUDE.md: AUV Pitch and Heading Hold (Julia model + Pixhawk IMU)

> Instructions for Claude (Claude Code) working in this repo. Read all of it before writing code.
> If something here conflicts with what the user asks, ask first.

## 0. The situation in five lines

1. Student capstone (VIT Chennai): **Real-Time Nonlinear Dynamics Modeling of a 4-DOF Autonomous Underwater Vehicle.**
2. A **working Julia model already exists** (notebook `AUV_4DOF_PJT_CODE_JULIA`). It is the **source of truth**. Do not rewrite the physics or "improve" the Lagrangian pipeline unless asked.
3. The vehicle is a torpedo hull: one rear thruster, **three servo fins**, a Pixhawk (IMU), a Raspberry Pi.
4. There is **no depth sensor**, so **depth hold is out**. The goal is **pitch hold and heading hold** using the Pixhawk IMU. Depth is *predicted by the model but not measured*.
5. Your job: (a) turn the notebook into reusable code, (b) design and test pitch/heading controllers on it in simulation, (c) connect the Pixhawk IMU, (d) log real runs, (e) compare model vs IMU and tune a few named parameters.

Keep everything simple, explainable and honest. A student must be able to defend every line in a review.

## 1. Hardware

| Part | Role |
|---|---|
| Pixhawk | IMU / attitude (pitch, yaw, rates) streamed over MAVLink. |
| Raspberry Pi | Reads the IMU, runs the controller, logs data. |
| 1 rear thruster + ESC | Forward thrust (set manually or constant). |
| 3 servo fins (Y-tail, 120° apart) | Pitch and yaw control. |
| Battery | Power. |

Not available: depth sensor, DVL, speed sensor, position sensor. Therefore **only pitch, pitch rate, yaw, yaw rate are measured.** Surge `x`, depth `z` and speed exist only in the model.
Do not use the Pixhawk's internal barometer for depth (it sits in a sealed hull and reads internal air pressure).

## 2. What the existing Julia code contains

Pipeline (keep these names): HTM `chains` → `make_params` → `build_kinematics` → `compute_KE` (+ added-mass fluid energy `T_fluid`) → `compute_Q_total` → `compute_M_h` → `SymbolicModel` → `compile_ode` → `dyn!`.
Equation solved: `M(q) q̈ + h(q, q̇) = Q`. Solver: `AutoTsit5(Rosenbrock23(autodiff = AutoFiniteDiff()))`, `reltol = abstol = 1e-6`.
Also present: `B(q,q̇) = jac(model.Q, [T_thr, δ1, δ2, δ3])` (actuator allocation, rank 3), `h_classes` (10 velocity-product classes), `run_switched` / `make_p` (switched ablation), `class_table`, plotting cells.

### Conventions (taken from the code; do not change)

- Generalized coordinates `q = [x, z, θ, ψ]`. State `u = [x, z, θ, ψ, ẋ, ż, θ̇, ψ̇]` (8 states).
- **World frame: z is UP.** `x` forward. Hull pose chain: `T_tran(x, 0, z) → T_rotz(ψ) → T_roty(θ)`.
- **θ positive = nose DOWN** (rotation about +y with z up). So nose-up means θ < 0.
- **ψ positive = counter-clockwise seen from above.**
- `y` is fixed at 0: **there is no sway DOF** (see Section 4, issue 2).
- Drag/damping coefficients are **negative** by convention (`F = Xu_*u + Xuu_*u|u|`).
- All inputs are entries of the parameter vector `p`: `T_thr` [N], `δ1, δ2, δ3` [rad]. Find indices with the existing `pidx(:name)`. **Never hard-code `p[15]`-style indices.**
- `p` layout: 21 config values (`config_names`), then mass, `Ix Iy Iz`, `cx cy cz`. Length 28. `length(p_syms) == length(p_vals)` must stay true.
- Fins: fin 1 = top (γ = 0), fins 2 and 3 = lower pair (γ = 120°, 240°).
  - `δ2 = −δ3 = +d` → **nose up** (θ decreases), almost no yaw.
  - `δ1 > 0` alone → ψ decreases.
  - Equal deflection on all three ≈ no pitch/yaw (it would be a roll torque, which the 4-DOF model does not have).
- Fin force ∝ dynamic pressure ∝ speed². `veps = 0.1` regularizes `1/u` and `v|v|` at rest. **No fin authority at near-zero speed.**
- `dyn!` freezes the state if `|θ| > 0.8 rad` or any state is non-finite. Keep that guard.

### Extracted allocation matrix (at u0, ẋ = 1 m/s, T = 8 N)

Rows = Q for `[x, z, θ, ψ]`, columns = `[T, δ1, δ2, δ3]`:

```
 0.999   0     -0.25   0.25
-0.05    0     -5.0    5.0
 0       0     -2.1    2.1
 0      -2.4    1.24   1.24
```

Rank 3. Heave and pitch rows are proportional in the fin columns, so **depth cannot be controlled independently of pitch**: it is "flown through attitude". For control, use the **2×3 fin block of the θ and ψ rows** (columns δ1..δ3), evaluated at the current speed (it scales with speed²).

### Placeholder values (NOT measured; flag them whenever you use them)

`m = 14 kg`, `com = [0,0,-0.02]`, `cb = [0,0,0.01]`, `Bb = 139 N` (weight 137.34 N, so +1.66 N net buoyancy), `ρw = 1025` (seawater!), inertia `[0.05, 1.0, 1.0]`, `x_fin = -0.42`, `R_fin = 0.09`, `r_thr = [-0.48,0,0]`, all drag/added-mass coefficients, `Afin = 0.004`, `CLa = 2.8`.
Real values must come from the SolidWorks mass-properties export and from tests (Section 6, Step 5). For a pool use `ρw ≈ 998` and recompute `Bb = ρ·g·V` consistently.

## 3. Golden numbers (regression tests for Step 0)

After extracting the model, these must reproduce (these are the notebook's current outputs):

| Test | Expected |
|---|---|
| `size(M) == (4,4)`, `length(h) == 4` | true |
| Statified `M_fun` vs substituted `M` | max difference ≈ 0 |
| Baseline: `u0 = [0,0,0.05,0,1,0,0,0]`, 30 s, `T = 8`, fins 0 | `retcode = Success`; x ≈ 25.01, z ≈ −0.3997, **θ ≈ 0.0911 rad**, ẋ ≈ 0.8289, ż ≈ −0.0138 |
| Thrust sweep from rest, 40 s | terminal ẋ: T=2 → 0.362, 5 → 0.633, 10 → 0.939, 15 → 1.173 m/s |
| Fin step, T=8, start ẋ=1: `δ2 = −δ3 = 0.2` | θ settles ≈ −0.14 rad (nose up), z rises |
| Fin step: `δ1 = 0.2` | ψ settles ≈ −0.10 rad |
| `rank(B)` at u0 | 3 |
| `K == 10` classes; `Σ parts − h` | ≤ ~1e-15 numerically |
| Switched ablation: `dxh^2` OFF | pitch trim drops ≈ 65% (0.0911 → ≈ 0.032) |
| `dynamics_sw!` all ON vs `compile_ode` | agree to ~1e-15 per RHS call |

If a refactor changes any of these, **stop and report**; do not silently adjust.

## 4. Known issues. Be honest about these in the report

1. **"Σ h_classes == model.h (expand): false."** The symbolic-expand check fails (expansion does not cancel to exactly 0). The *numerical* checks pass at round-off (~1e-15). Do not call it a "symbolic proof"; call it "numerically verified to round-off". Optionally investigate (e.g. simplify before comparing) but do not hide it.
2. **No sway DOF (`y = 0`).** The hull yaws but its origin still moves along world-x only, so the fin cross-flow term pulls ψ back toward 0. Pitch dynamics (vertical plane) are fine. **Yaw/heading dynamics are an approximation** and will not match a real vehicle that drifts sideways. Compare yaw only over short windows (fin-step response) and say so. Optional upgrade: add `y` as a 5th coordinate (`T_tran(q1, q5, q2)`) with sway added mass and drag; the generic pipeline should allow it.
3. **Yaw plots at ~1e-16 are round-off, not physics** (symmetric run, no yaw input). Don't present them as results.
4. `ρw = 1025` is seawater, placeholders everywhere (Section 2).
5. Pluto notebooks are reactive. Cell order does not matter there but **does** matter in plain `.jl` files: `states_dync`, `statify`, `chains`, `cfg` and `p_vals` are globals used by later cells. Preserve dependency order when extracting.
6. The CAD and code have **3 fins**. Some slides say 4. Use 3.

## 5. Scope

**In scope**
1. Package the model as reusable Julia code with the regression tests above.
2. Closed-loop simulation: pitch hold + heading hold + fin allocator, using the model.
3. Pixhawk IMU interface on the Raspberry Pi (pitch, yaw, rates) with correct sign conversion.
4. Run controllers on the real vehicle, log everything.
5. **Offline replay:** feed the logged commands into the Julia model, compare predicted vs measured θ and ψ, tune a few named parameters.

**Out of scope (unless explicitly asked)**
- Depth hold, waypoint navigation (need depth/position sensing).
- 6-DOF, CFD, RL, MPC, Kalman-filter research, vision.
- Running the symbolic derivation on the Pi. Do **not** claim a real-time on-vehicle model unless one is truly running; "offline replay of logged runs" is the honest default.

## 6. Build plan (one step at a time; each step must run and be checked before the next)

**Step 0: Extract and lock the model**
- `src/auv_model.jl`: notebook cells copied *verbatim*, in dependency order, no behavior change. Wrap in a function returning `(model, p_vals, pidx, idx_T, idx_δ1, idx_δ2, idx_δ3, dyn!, B_fun, K, labels)`.
- `test/golden.jl` implements the table in Section 3. Keep the original notebook untouched.

**Step 1: Closed-loop simulation**
- Control loop at 20 Hz using a `PeriodicCallback` (or a manual fixed-step loop) that writes `δ1..δ3` into `integrator.p` (via `pidx`). Thrust is constant per run.
- Pitch controller (inner): PID on `θ_ref − θ`, D-term from `θ̇` (the gyro rate, never a differentiated angle), integral with anti-windup. Output is a pitch command.
- Heading controller: PID on wrapped `ψ_ref − ψ`, D from `ψ̇`. Output is a yaw command.
- **Fin allocator** (`control/allocator.jl`): take the 2×3 fin block `Bf` of the θ and ψ rows from `B_fun(u, p)`; solve `Bf * δ = [τθ; τψ]` with the pseudo-inverse; saturate each fin at ±20° (placeholder, ask user for servo limits); rate-limit.
- Note the natural trim: with fins centred the hull settles at θ ≈ 0.091 rad (Munk moment + CG/CB offset). The integrator or a feed-forward trim must cancel this. Hold θ_ref = 0 as the first test.
- Tests: ±5° pitch step; heading step (short window, see issue 2); disturbance (change `Bb` by ±2%, or shift `com`); speed sweep T = 5, 8, 12 N; behavior at low speed (must degrade gracefully, not blow up); fin saturation.
- Report: overshoot, settling time, steady-state error, max fin angle.

**Step 2: Pixhawk IMU interface (Python on the Pi)**
- `pymavlink`, read MAVLink `ATTITUDE` (roll, pitch, yaw, rates). Apply the conversions in Section 7 in **one** function with a unit test.
- Provide a simulated IMU source so the controller runs on a laptop with no hardware.
- Do not assume the Pixhawk firmware (ArduSub / ArduPilot / PX4). See Section 11 questions.

**Step 3: Controller runtime on the Pi**
- Python is fine on the Pi. Julia exports small tables so the Pi never runs Symbolics:
  `scripts/export_for_pi.jl` writes `export/allocation_table.json` (pseudo-inverse of the fin block for speeds 0.2 to 1.5 m/s) and `export/thrust_speed_map.json` (from the thrust sweep) plus the PID gains.
- **There is no speed sensor**, so schedule the allocator on the *nominal* speed from the thrust map. Below a minimum speed, command neutral fins and print a warning.
- Fail-safe (Section 9) lives here.

**Step 4: Pool tests and logging**
CSV columns: `t, thrust_cmd, d1, d2, d3, pitch_meas, yaw_meas, pitch_rate, yaw_rate, roll_meas, mode`. Log roll too, even though the model has no roll. Tests, in order:
1. **Bench (in air):** tilt the hull by hand, confirm signs of pitch/yaw/fin directions end to end.
2. **Static trim / ballast:** make buoyancy slightly positive (vehicle surfaces if power is lost). The resting pitch gives information on the CG/CB offset.
3. **Free pitch decay (thrust off):** nudge the nose, log the oscillation → natural frequency and damping (constrains restoring moment, `Mqd`, `Mq`).
4. **Fixed thrust, fins centred:** steady pitch trim vs model. Time a known pool length for average speed → check terminal speed vs thrust.
5. **Fin steps:** pitch step (`δ2 = −δ3`) and yaw step (`δ1`) → compare step responses.
6. **Closed-loop pitch hold**, then **heading hold**.

**Step 5: Offline replay and tuning**
- `analysis/replay.jl`: initialize the model from the measured first `θ, θ̇, ψ, ψ̇`, feed logged `T, δ1..δ3` as time-varying inputs, integrate, overlay predicted vs measured θ and ψ, print RMS error over sliding windows.
- Tune **only named parameters**, one or two at a time, and record each change: `Mq, Mqq, CLa, Afin, Xuu_, Bb, com/cb offset, Zwd (Munk), Mqd`. Prefer a small manual or least-squares fit. Never fit everything at once. Write which parameter changed, from what to what, and why.
- Stretch: stream the live IMU to a laptop that runs the Julia model for a live comparison. Label it "laptop-in-the-loop", not "on-vehicle".

**Step 6: Deliverables**
Comparison plots (θ and ψ, predicted vs measured), a table of tuned parameters, closed-loop step-response metrics, the switched-ablation figure (Munk moment dominates trim), and a README that reproduces everything from a fresh clone.

## 7. Pixhawk ↔ model sign conversions (get these right; test on the bench)

MAVLink/ArduPilot attitude is NED/FRD: **pitch positive nose UP**, **yaw positive clockwise seen from above**, yaw in (−π, π] (heading from north). The model is the opposite on both.

| Quantity | Model value |
|---|---|
| Pitch | `θ = −pitch_mav` |
| Pitch rate | `θ̇ = −pitchspeed_mav` |
| Yaw | `ψ = −wrap(yaw_mav − yaw_mav_at_start)` (wrap to (−π, π]) |
| Yaw rate | `ψ̇ = −yawspeed_mav` |

- Valid for small angles (|θ| < ~0.3 rad), where body rates ≈ Euler rates.
- Assumes the Pixhawk axes align with the hull (x forward, y right, z down). If mounted differently, apply one constant rotation in one place.
- Yaw comes from a compass inside a hull next to a thruster: check on the bench that yaw stays steady with the thruster running. If not, use a relative gyro-based heading and say so.

## 8. Repo layout

```
auv/
├── CLAUDE.md
├── README.md
├── notebooks/AUV_4DOF_PJT_CODE_JULIA.jl   # original, untouched
├── src/auv_model.jl                       # verbatim extraction
├── test/golden.jl                         # Section 3
├── control/                               # PID, allocator, closed-loop sim
├── scripts/export_for_pi.jl
├── export/                                # JSON tables for the Pi
├── pi/                                    # imu_mavlink.py, controller.py, actuators.py, logger.py
├── logs/                                  # CSV runs (small files only)
├── analysis/                              # replay.jl, plots
└── docs/                                  # conventions.md, wiring.md, results.md
```

## 9. Safety (hardware can be damaged or lost)

- If IMU data is missing for more than 0.5 s → thruster off, fins neutral.
- If `|pitch| > 0.6 rad` or loss of control is detected → thruster off, fins neutral, log it.
- Fin angle and rate limits enforced in **one** function, used by sim and hardware alike.
- Arm/disarm switch; thruster default off; first runs tethered and in a shallow tank.
- Positive buoyancy so that failure means "floats up".
- Before every in-water run: bench sign check (Section 6, Step 4.1).

## 10. Coding rules

- Short, readable, commented: say *why*, not just *what*. Units in names (`pitch_rad`, `depth_m`).
- Radians inside the model and controller. Degrees only for display.
- No magic numbers: gains, limits, speeds, parameters live in one config file each.
- Never invent measurements or results. If data is missing, say so, and mark placeholders loudly.
- Do not add heavy dependencies without asking; list any new dependency and why.
- After each step tell the user: what was built, how to run it, which test passed, what is still a placeholder.

## 11. Ask the user before you code (do not guess)

1. Pixhawk firmware (ArduSub / ArduPilot / PX4), which port, and baud rate to the Pi?
2. **Who drives the servos and the ESC?** Pixhawk outputs, an ESP32, or the Pi? (This decides `actuators.py`.)
3. Servo angle limits and which direction is positive for each fin.
4. Real mass, CG, CB, inertia from the SolidWorks mass-properties export.
5. Fresh water (pool) or seawater? Is the thruster ESC bidirectional?
6. Julia version, and is the notebook a Pluto notebook?

## 12. Definition of done

- [ ] Extracted model passes every row of Section 3.
- [ ] Closed-loop sim: pitch hold and heading hold work, with reported overshoot, settling time and steady-state error, including at reduced speed and with saturation.
- [ ] IMU conversions are unit-tested and verified on the bench.
- [ ] At least one real pitch-hold run with logs.
- [ ] One figure of predicted vs measured θ (and ψ over a short window) from a real run, with RMS error.
- [ ] A table of every parameter changed during tuning, with the reason.
- [ ] Report states clearly: depth and speed are model predictions, not measurements; heading is an approximation (no sway); symbolic-proof cell issue noted.

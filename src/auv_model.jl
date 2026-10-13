# ============================================================================
# 4-DOF AUV model: extraction of the Pluto notebook AUV_4DOF_PJT_CODE_JULIA.
#
# The notebook is the source of truth. The cells below are copied from it in
# DEPENDENCY order (Pluto is reactive, a plain .jl file is not), with no change
# to the physics. Differences from the notebook, all deliberate:
#   - Plotting cells (WGLMakie) and the SymPy helpers (sympy_simplify,
#     sympylify) are left out: nothing in the model depends on them.
#   - `using DifferentialEquations` -> `using OrdinaryDiffEq` (same solvers,
#     far smaller install). AutoFiniteDiff comes from ADTypes.
#   - Cells that only run a simulation or print a check live in test/golden.jl.
#   - The cells are wrapped in `module AUVModel`, and `auv_model()` at the
#     bottom returns the pieces other code needs.
#
# The notebook was transcribed from a PDF export (images, no text layer), so
# test/golden.jl is the proof that the transcription is right.
#
# WARNING: every mass, geometry and hydrodynamic number below is a PLACEHOLDER
# (not measured). See CLAUDE_AUV_PJT.md Section 2.
# ============================================================================
module AUVModel

# ========= CORE SYMBOLICS =========
using Symbolics
using LinearAlgebra

# ========= DYNAMICS / SOLVER =========
using OrdinaryDiffEq
using ADTypes: AutoFiniteDiff

# ---------------------------------------------------------------------------
@variables t

Td = Differential(t)

# scalar time derivative
TD(x) = expand_derivatives(Td(x))

# vector/matrix time derivative (broadcast elementwise)
TDv(X) = expand_derivatives.(Td.(X))

# aliases
jac  = Symbolics.jacobian
hess = Symbolics.hessian

# homogeneous conversion: [x,y,z] -> [x,y,z,1]
function to_hom(v)
    length(v) == 4 ? Num.(v) : Num[v[1], v[2], v[3], 1]
end

# ---------------------------------------------------------------------------
function eye3()
    Num[
        1 0 0
        0 1 0
        0 0 1
    ]
end

function eye4()
    Num[
        1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1
    ]
end

function T_tran(x, y, z)
    Num[
        1 0 0 x
        0 1 0 y
        0 0 1 z
        0 0 0 1
    ]
end

T_tran(v) = T_tran(v[1], v[2], v[3])   # multiple dispatch on a vector

function T_rotx(θ)
    Num[
        1 0       0      0
        0 cos(θ) -sin(θ) 0
        0 sin(θ)  cos(θ) 0
        0 0       0      1
    ]
end

function T_roty(θ)
    Num[
         cos(θ) 0 sin(θ) 0
         0      1 0      0
        -sin(θ) 0 cos(θ) 0
         0      0 0      1
    ]
end

function T_rotz(θ)
    Num[
        cos(θ) -sin(θ) 0 0
        sin(θ)  cos(θ) 0 0
        0       0      1 0
        0       0      0 1
    ]
end

# ---------------------------- 3. chain = {T1, T2, …, TN} --------------------
# A chain is a Vector of 4×4 HTMs:
# chain = [T_tran(q[1], q[2], q[3]), T_rotx(q[4]), T_roty(q[5])]
#
# Pose = T1 * T2 * ... * TN

function build_pose(chain::Vector)
    prod(chain)
end

# ---------------------------- inertia tensor helpers ------------------------

# Principal / diagonal inertia tensor
function inertia_principal(Ix, Iy, Iz)
    Num[
        Ix  0   0
        0   Iy  0
        0   0   Iz
    ]
end

# Full symmetric inertia tensor with products of inertia
function inertia_tensor(Ix, Iy, Iz; Ixy=0, Ixz=0, Iyz=0)
    Num[
        Ix   Ixy  Ixz
        Ixy  Iy   Iyz
        Ixz  Iyz  Iz
    ]
end

# Symbolic principal inertia for one body
function make_I_principal(body_id)
    Ix = Symbolics.variable(Symbol("I_x" * string(body_id)))
    Iy = Symbolics.variable(Symbol("I_y" * string(body_id)))
    Iz = Symbolics.variable(Symbol("I_z" * string(body_id)))

    vars = Num[Ix, Iy, Iz]

    Ibody = inertia_principal(Ix, Iy, Iz)

    return vars, Ibody
end

# Symbolic full inertia tensor for one body
function make_I_tensor(body_id)
    Ix  = Symbolics.variable(Symbol("I_x"  * string(body_id)))
    Iy  = Symbolics.variable(Symbol("I_y"  * string(body_id)))
    Iz  = Symbolics.variable(Symbol("I_z"  * string(body_id)))

    Ixy = Symbolics.variable(Symbol("I_xy" * string(body_id)))
    Ixz = Symbolics.variable(Symbol("I_xz" * string(body_id)))
    Iyz = Symbolics.variable(Symbol("I_yz" * string(body_id)))

    vars = Num[Ix, Iy, Iz, Ixy, Ixz, Iyz]

    Ibody = inertia_tensor(
        Ix, Iy, Iz;
        Ixy = Ixy,
        Ixz = Ixz,
        Iyz = Iyz
    )

    return vars, Ibody
end

# Generic inertia maker
# mode = :principal → only I_x, I_y, I_z
# mode = :full      → I_x, I_y, I_z, I_xy, I_xz, I_yz
function make_inertia(body_id; mode = :full)

    if mode == :principal

        return make_I_principal(body_id)

    elseif mode == :full

        return make_I_tensor(body_id)

    end
end

# ---------------------------- 7. bodies / forces / system -------------------
# ── force / torque constructors ──────────────────────────────
# CONVENTION:
#   point_local        → always in that body's OWN local frame
#   force/torque vector → frame keyword decides:
#       frame = :world → world components (default)
#       frame = :body  → components in the body's own local frame
#                        (resolved to world inside compute_Q_total via kin.R)
#   For frame_force / frame_torque (two bodies):
#       frame = :world | :body1 | :body2
point_force(body_id::Int, point_local, force_vector; frame = :world) =
    (; body_id,
       point_local  = Num.(point_local),
       force_vector = Num.(force_vector),
       frame)

body_torque(body_id::Int, torque_vector; frame = :world) =
    (; body_id,
       torque_vector = Num.(torque_vector),
       frame)

frame_force(body1_id::Int, point1_local,
            body2_id::Int, point2_local,
            force_vector; frame = :world) =
    (; body1_id,
       point1_local = Num.(point1_local),
       body2_id,
       point2_local = Num.(point2_local),
       force_vector = Num.(force_vector),
       frame)

frame_torque(body1_id::Int, body2_id::Int, torque_vector; frame = :world) =
    (; body1_id,
       body2_id,
       torque_vector = Num.(torque_vector),
       frame)

# ── body definition — loads attached to the body ─────────────
make_body(id::Int, chain::Vector; forces = [], torques = []) =
    (; id, chain, forces, torques)

# ── pose access ──────────────────────────────────────────────
body_pose(body) = build_pose(body.chain)
system_poses(sys) = Dict(b.id => body_pose(b) for b in sys.bodies)

# ── system assembly — loads collected off the bodies ─────────
multibody_system(bodies; frame_forces = [], frame_torques = []) =
    (; bodies,
       point_forces = reduce(vcat, [b.forces  for b in bodies]; init = []),
       body_torques = reduce(vcat, [b.torques for b in bodies]; init = []),
       frame_forces,
       frame_torques)

# ---------------------------- Generic Parameter Block P ---------------------
function make_params(body_ids::Vector{Int};
                     config = Symbol[],
                     config_values = [],
                     stiff_damp = Symbol[],
                     misc = Symbol[],
                     inertia_mode = :full)

    N = length(body_ids)

    # ── configuration parameters ─────────────────────────────
    config_params = (; (s => Symbolics.variable(s) for s in config)...)
    config_syms = Num[getfield(config_params, s) for s in config]
    config_value_dict = Dict(config_syms[i] => config_values[i]
                             for i in eachindex(config_values))

    # ── inertial parameters, auto-generated per body ─────────
    M_body = Num[Symbolics.variable(Symbol("m_" * string(id))) for id in body_ids]

    I_body = Vector{Matrix{Num}}(undef, N)
    I_vars = Vector{Vector{Num}}(undef, N)
    for (j, id) in enumerate(body_ids)
        vars, Ib = make_inertia(id; mode = inertia_mode)
        I_vars[j] = vars
        I_body[j] = Ib
    end

    com = [Num[Symbolics.variable(Symbol("c_x" * string(id))),
               Symbolics.variable(Symbol("c_y" * string(id))),
               Symbolics.variable(Symbol("c_z" * string(id)))]
           for id in body_ids]

    # ── stiffness / damping ──────────────────────────────────
    stiff_damp_params = (; (s => Symbolics.variable(s) for s in stiff_damp)...)
    stiff_damp_syms = Num[getfield(stiff_damp_params, s) for s in stiff_damp]

    # ── miscellaneous ────────────────────────────────────────
    misc_params = (; (s => Symbolics.variable(s) for s in misc)...)
    misc_syms = Num[getfield(misc_params, s) for s in misc]

    # ── main parameter structure (no g — gravity is a force input) ─
    P = (; N, body_ids,
           config = config_params, config_values, config_value_dict,
           M_body, I_body, com,
           stiff_damp = stiff_damp_params,
           misc = misc_params)

    # ── flat symbolic parameter vector ───────────────────────
    # order: config | body-wise (m, I vars, com) | stiff_damp | misc
    p_syms = Num[]
    append!(p_syms, config_syms)

    inertial_syms = Num[]
    for j in 1:N
        push!(inertial_syms, M_body[j])
        append!(inertial_syms, I_vars[j])
        append!(inertial_syms, com[j])
    end
    append!(p_syms, inertial_syms)

    append!(p_syms, stiff_damp_syms)
    append!(p_syms, misc_syms)

    # ── grouped view for building numeric values / debugging ──
    param_inputs = (; config = config_syms,
                      inertial = inertial_syms,
                      stiff_damp = stiff_damp_syms,
                      misc = misc_syms)

    return P, p_syms, param_inputs
end

# ---------------------------------------------------------------------------
# inverse of skew: pulls ω out of a skew-symmetric matrix
function vee_skew(S)
    Num[
        S[3, 2]
        S[1, 3]
        S[2, 1]
    ]
end

# world-frame angular velocity of a body given its 4x4 pose T
function Omega(T)
    R    = T[1:3, 1:3]
    Rdot = TDv(R)
    Ω    = (Rdot * transpose(R))   # Ṙ Rᵀ = skew(ω_world)
    ω    = (vee_skew(Ω))
    return ω
end

# ---------------------------------------------------------------------------
function build_kinematics(sys, q, P)
    B = length(sys.bodies)
    T_body_list = Vector{Matrix{Num}}(undef, B)
    R_list      = Vector{Matrix{Num}}(undef, B)
    rG_list     = Vector{Vector{Num}}(undef, B)
    vG_list     = Vector{Vector{Num}}(undef, B)
    ω_list      = Vector{Vector{Num}}(undef, B)
    Iglob_list  = Vector{Matrix{Num}}(undef, B)
    for i in 1:B
        body = sys.bodies[i]
        # Body pose from HTM chain
        T_body = build_pose(body.chain)
        T_body_list[i] = T_body
        # Rotation matrix of body
        R = T_body[1:3, 1:3]
        R_list[i] = R
        # COM position in world frame (P.com[i] is body-local)
        rG = (T_body * to_hom(P.com[i]))[1:3]
        rG_list[i] = rG
        # COM velocity in world frame
        vG = TDv(rG)
        vG_list[i] = vG
        # Angular velocity in world frame
        ω_list[i] = Omega(T_body)
        # Inertia tensor expressed in world frame
        Iglob_list[i] = R * P.I_body[i] * transpose(R)
    end
    return (;
        T_body = T_body_list,
        R      = R_list,
        rG     = rG_list,
        vG     = vG_list,
        omega  = ω_list,
        Iglob  = Iglob_list
    )
end

# =============== 8. KINETIC ENERGY (König) ===============
#
# For each rigid body:
#
#   KE_i = 1/2 m_i vG_iᵀ vG_i  +  1/2 ω_iᵀ I_world_i ω_i
#
# m_i from P.M_body (auto-generated symbols)
# vG, ω, Iglob from build_kinematics — all world frame, consistent pair
#
function KE_body(m, I_global, vG, ω)
    KE_tr  = (1//2) * m * dot(vG, vG)
    KE_rot = (1//2) * dot(ω, I_global * ω)
    return KE_tr + KE_rot, KE_tr, KE_rot
end

function compute_KE(sys, P, kin)
    N = length(sys.bodies)
    KE_each     = Vector{Num}(undef, N)
    KE_tr_each  = Vector{Num}(undef, N)
    KE_rot_each = Vector{Num}(undef, N)
    KE_total = Num(0)
    for i in 1:N
        KE_i, KE_tr, KE_rot = KE_body(
            P.M_body[i],
            kin.Iglob[i],
            kin.vG[i],
            kin.omega[i]
        )
        KE_each[i]     = KE_i
        KE_tr_each[i]  = KE_tr
        KE_rot_each[i] = KE_rot
        KE_total += KE_i
    end
    return (;
        KE_total,
        KE_each,
        KE_trans = KE_tr_each,
        KE_rot   = KE_rot_each
    )
end

# =============== 9. GENERALIZED FORCES (virtual work) ===============
#
# Q_total = Σ point forces + Σ body torques
#         + Σ frame forces + Σ frame torques
#
# All computation in world frame; :body vectors rotated to world

function compute_Q_total(sys, q, kin)

    n    = length(q)
    qdot = TD.(q)

    Q_total = zeros(Num, n)

    # ── 1. POINT FORCES:  Q = Jᵀ F,  J = ∂r_point/∂q ─────────
    for f in sys.point_forces
        T_body  = kin.T_body[f.body_id]
        r_point = (T_body * to_hom(f.point_local))[1:3]
        F_world = f.frame == :body  ? kin.R[f.body_id] * f.force_vector :
                                      f.force_vector
        J_point = jac(r_point, q)
        Q_total .+= vec(transpose(J_point) * F_world)
    end

    # ── 2. BODY TORQUES:  Q = Jωᵀ τ,  Jω = ∂ω/∂q̇ ────────────
    for τ in sys.body_torques
        ω_body  = kin.omega[τ.body_id]
        τ_world = τ.frame == :body  ? kin.R[τ.body_id] * τ.torque_vector :
                                      τ.torque_vector
        Jω = jac(ω_body, qdot)
        Q_total .+= vec(transpose(Jω) * τ_world)
    end

    # ── 3. FRAME FORCES (action-reaction between two points) ──
    #    F on body2 at p2, -F on body1 at p1, via r_rel = p2 - p1
    for ff in sys.frame_forces
        p1 = (kin.T_body[ff.body1_id] * to_hom(ff.point1_local))[1:3]
        p2 = (kin.T_body[ff.body2_id] * to_hom(ff.point2_local))[1:3]
        F_world = ff.frame == :body1 ? kin.R[ff.body1_id] * ff.force_vector :
                  ff.frame == :body2 ? kin.R[ff.body2_id] * ff.force_vector :
                                       ff.force_vector
        J_rel = jac(p2 .- p1, q)
        Q_total .+= vec(transpose(J_rel) * F_world)
    end

    # ── 4. FRAME TORQUES (relative couple between two bodies) ─
    #    τ on body2, -τ on body1, via ω_rel = ω2 - ω1
    for ft in sys.frame_torques
        ω_rel   = kin.omega[ft.body2_id] .- kin.omega[ft.body1_id]
        τ_world = ft.frame == :body1 ? kin.R[ft.body1_id] * ft.torque_vector :
                  ft.frame == :body2 ? kin.R[ft.body2_id] * ft.torque_vector :
                  ft.frame == :world ? ft.torque_vector :
                                       ft.torque_vector
        Jω_rel = jac(ω_rel, qdot)
        Q_total .+= vec(transpose(Jω_rel) * τ_world)
    end

    return Q_total
end

# =============== 10. MASS MATRIX AND h VECTOR ===============
#
# Euler-Lagrange kinetic part:
#
#   EL = d/dt(∂T/∂q̇) - ∂T/∂q  =  M(q) q̈ + h(q, q̇)
#
# Gravity and applied loads are NOT here — they live in Q_total.
# Final dynamics:  M q̈ + h = Q   ⇒   q̈ = M⁻¹(Q - h)
function compute_M_h(KE_total, q; method = :hessian)
    qdot  = TD.(q)
    qddot = TD.(qdot)

    # ── Euler-Lagrange kinetic part ──────────────────────────
    dKE_dqdot = vec(jac([KE_total], qdot))     # ∂T/∂q̇
    dKE_dq    = vec(jac([KE_total], q))        # ∂T/∂q
    EL = TDv(dKE_dqdot) .- dKE_dq

    # ── mass matrix ──────────────────────────────────────────
    if method == :hessian
        # T exactly quadratic in q̇ ⇒ Hessian is M (symmetric)
        M = hess(KE_total, qdot)
    elseif method == :accelcoeff
        # EL linear in q̈ ⇒ coefficient matrix is M
        M = jac(EL, qddot)
    end

    # ── h by algebraic cancellation (robust, self-checking) ──
    # EL = M q̈ + h identically ⇒ h = EL - M q̈ ; the q̈ terms
    # cancel by expansion. If any q̈ survives in h, extraction
    # of M was wrong — visible on inspection, never silent.
    # ── h: substitute q̈ = 0 into EL (no expansion — memory-safe) ──
    zero_acc = Dict(qddot .=> 0)
    h_vec = substitute.(EL, Ref(zero_acc))

    return M, h_vec
end

# =============== 11. THE SYMBOLIC MODEL ===============
struct SymbolicModel
    sys       # multibody system (bodies + loads)
    P         # parameter block (M_body, I_body, com, config…)
    q         # generalized coordinates
    qdot      # generalized velocities
    x         # state vector [q; qdot]
    p_syms    # ordered symbolic parameter vector (build_function order)
    kin       # kinematics table
    KE        # kinetic energy data
    Q         # total generalized force vector
    M         # mass matrix
    h         # nonlinear vector
end

function build_symbolic_model(sys, q, P, p_syms; method = :hessian)
    qdot = TD.(q)
    x    = vcat(q, qdot)
    kin = build_kinematics(sys, q, P)
    KE_data = compute_KE(sys, P, kin)
    Q_total = compute_Q_total(sys, q, kin)
    M, h_vec = compute_M_h(KE_data.KE_total, q; method = method)
    return SymbolicModel(sys, P, q, qdot, x, p_syms,
                         kin, KE_data, Q_total, M, h_vec)
end

# ======== REPLACED CELL — 4-DOF AUV generalized coordinates ========
#
#   q = [ xh  zh  θh  ψh ]  =  [ surge pos, heave pos, pitch, yaw ]
#
# Sway and roll are dropped: torpedo hull is metacentrically stable in
# roll (CB above CG) and has no sway actuation. World frame: z UP.
@variables xh(t) zh(t) θh(t) ψh(t)
q = Num[xh, zh, θh, ψh]

# ======== statify / destatify — the symbolic boundaries ========
#
#   statify   : ψ(t), D(ψ(t))  →  plain q_dummy[i]   (for build_function)
#   destatify : the exact inverse
#
subs(D) = x -> substitute(x, D; fold = Val(true))

states_dyn = Num[q...; TD.(q)...]
n_state = length(states_dyn)

@variables q_dummy[1:n_state]
states_dync = collect(q_dummy)

statify_dict   = Dict(zip(states_dyn,  states_dync))
destatify_dict = Dict(zip(states_dync, states_dyn))

statify(expr)   = expr |> subs(statify_dict)
destatify(expr) = expr |> subs(destatify_dict)

# =============== 12. COMPILE + SIMULATE ===============
#
# statify (global, boundary cell): ψ(t), D(ψ(t)) → q_dummy[i]
# build_function: THE symbolic → numeric boundary
#
#   u = [q; qdot]   — ordering fixed by states_dyn in the boundary cell
#   p = numeric values in the order of model.p_syms  (= p_vals)

function compile_ode(model)
    n = length(model.q)

    # ── symbolic → plain-variable form ───────────────────────
    M_s = statify.(model.M)
    h_s = statify.(model.h)
    Q_s = statify.(model.Q)

    # ── compile to numeric functions ─────────────────────────
    M_fun = build_function(M_s, states_dync, model.p_syms; expression = Val(false), cse = true)[1]
    h_fun = build_function(h_s, states_dync, model.p_syms; expression = Val(false), cse = true)[1]
    Q_fun = build_function(Q_s, states_dync, model.p_syms; expression = Val(false), cse = true)[1]

    # ── ODE right-hand side:  M q̈ = Q - h ───────────────────
    function dynamics!(du, u, p, tnow)
        M_now = M_fun(u, p)
        rhs   = vec(Q_fun(u, p)) .- vec(h_fun(u, p))
        du[1:n]    .= u[n+1:2n]
        du[n+1:2n] .= M_now \ rhs
        return nothing
    end
    return dynamics!
end

function run_simulation(model, u0, tspan;
                        p, solver = Tsit5(),
                        reltol = 1e-8, abstol = 1e-8)
    prob = ODEProblem(compile_ode(model), u0, tspan, p)
    return solve(prob, solver; reltol = reltol, abstol = abstol)
end

# ======== REPLACED CELL — one body, one chain (was 6 chains) ========
#
#   hull pose:  translate (x, 0, z) → yaw ψ about world z → pitch θ about hull y
#
chains = [
    [T_tran(q[1], 0, q[2]), T_rotz(q[4]), T_roty(q[3])]
]

# ======== REPLACED CELL — hull geometry (plain numbers only) ========
# ⚠ PLACEHOLDER values sized for the CAD torpedo (~0.9 m, Ø0.15 m).
#   Measure the real ones from SolidWorks mass properties and replace.
m_hull_val = 14.0                       # kg
W_val      = m_hull_val * 9.81          # 137.34 N
com_val    = [0.0, 0.0, -0.02]          # CG 2 cm BELOW hull origin (body frame, z up)
cb_val     = [0.0, 0.0,  0.01]          # CB 1 cm ABOVE origin → pitch restoring

r_thr      = [-0.48, 0.0, 0.0]          # thruster on axis, at the tail
x_fin      = -0.42                      # fin axial station
R_fin      = 0.09                       # fin mount radius off the hull axis

# ======== REPLACED CELL — actuator + hydrodynamic force inputs ========
#
# cfg(s) creates the IDENTICAL symbol make_params regenerates from
# config_names later — same trick that made Vt/kd sweepable through p.
cfg(s) = Symbolics.variable(s)

T_thr_s = cfg(:T_thr)                          # thruster force [N]
δ1_s = cfg(:δ1); δ2_s = cfg(:δ2); δ3_s = cfg(:δ3)   # the 3 servo fins [rad]
ρw = cfg(:ρw); Bb = cfg(:Bb); veps = cfg(:veps)
Xu_ = cfg(:Xu_); Xuu_ = cfg(:Xuu_); Zw_ = cfg(:Zw_); Zww_ = cfg(:Zww_)
Mq_ = cfg(:Mq_); Mqq_ = cfg(:Mqq_); Nr_ = cfg(:Nr_); Nrr_ = cfg(:Nrr_)
Afin = cfg(:Afin); CLa = cfg(:CLa)

# ── hull kinematics for force construction (pure chain algebra) ──
T_hull = build_pose(chains[1])
R_hull = T_hull[1:3, 1:3]

# body-frame velocity of any hull-local point
vbody(pl) = R_hull' * TDv((T_hull * to_hom(pl))[1:3])

vo   = vbody([0, 0, 0])               # hull-origin velocity, body frame
u_b  = vo[1]                          # surge
w_b  = vo[3]                          # heave
qp_b = TD(q[3])                       # pitch rate
r_b  = TD(q[4])                       # yaw rate

# ── THE regularization — verbatim from the 9-DOF tyre model ──
inv_u   = u_b / (u_b^2 + veps^2)      # smooth 1/u, finite at rest
sabs(v) = v * sqrt(v^2 + veps^2)      # smooth v·|v| for quadratic drag
qdyn    = (1//2) * ρw * (u_b^2 + veps^2)   # fin dynamic pressure

# ── 3 servo fins at 120° around the tail (Y-tail, from the CAD) ──
#   γ measured from +z around the hull x-axis:
#   fin 1 = top (the dorsal fin in the front view), fins 2,3 = lower pair
#   span direction r̂ = [0, sinγ, cosγ] ;  lift direction t̂ = x̂ × r̂
γ_fins = [0.0, 2π/3, 4π/3]
δ_fins = [δ1_s, δ2_s, δ3_s]
fin_t(γ) = Num[0, cos(γ), -sin(γ)]
fin_r(γ) = [x_fin, R_fin*sin(γ), R_fin*cos(γ)]

fin_forces = [begin
    γ  = γ_fins[j]
    t̂  = fin_t(γ)
    rf = fin_r(γ)
    vt = dot(vbody(rf), t̂)            # local cross-flow along lift dir
    αj = δ_fins[j] - vt * inv_u       # deflection + weathervane term
    (Num.((qdyn * Afin * CLa * αj) .* t̂), rf, 1, :body)
end for j in 1:3]

# ── force_inputs: (force_vector, point_local, body_id, frame) ──
force_inputs = vcat(Any[
    ([0.0, 0.0, -W_val],  com_val,   1, :world),   # gravity at CG
    ([0.0, 0.0,  Bb],     cb_val,    1, :world),   # buoyancy at CB (restoring pair)
    ([T_thr_s, 0.0, 0.0], r_thr,     1, :body),    # thruster, along hull axis
    ([Xu_*u_b + Xuu_*sabs(u_b), 0.0,
      Zw_*w_b + Zww_*sabs(w_b)], [0,0,0], 1, :body),  # translational drag
], fin_forces)

# ── torque_inputs: (torque_vector, body_id, frame) ──
torque_inputs = [
    ([0.0, Mq_*qp_b + Mqq_*sabs(qp_b), 0.0], 1, :body),   # pitch damping
    ([0.0, 0.0, Nr_*r_b + Nrr_*sabs(r_b)],   1, :body),   # yaw damping
]

# ======== REPLACED CELL — the parameter values (order = config_names) ========
# ⚠ Hydrodynamic coefficients are literature-plausible placeholders for a
#   small torpedo AUV — calibrate against CFD / tow-tank data when available.
inertia_mode  = :principal

config_names  = [:T_thr, :δ1, :δ2, :δ3,
                 :ρw, :Bb, :veps,
                 :Xu_, :Xuu_, :Zw_, :Zww_, :Mq_, :Mqq_, :Nr_, :Nrr_,
                 :Xud, :Zwd, :Mqd, :Nrd,
                 :Afin, :CLa]

config_values = [8.0, 0.0, 0.0, 0.0,          # thrust 8 N, fins centered
                 1025.0, 139.0, 0.10,          # ρ, buoyancy (>W: floats up), v_eps
                 -2.0, -9.0, -15.0, -40.0,     # surge/heave drag (lin, quad)
                 -1.5, -4.0, -1.5, -4.0,       # pitch/yaw damping (lin, quad)
                 -1.0, -12.0, -1.2, -1.2,      # added mass Xu̇, Zẇ, Mq̇, Nṙ
                 0.004, 2.8]                   # fin area, lift slope

mass_values    = [m_hull_val]
inertia_values = [[0.05, 1.0, 1.0]]            # slender body: Ixx « Iyy = Izz
com_values     = [com_val]

# =========== PROCESSING: build parameters, bodies, system ===========
B = length(chains)

P, p_syms, param_inputs = make_params(collect(1:B);
    config = config_names, config_values = config_values,
    inertia_mode = inertia_mode)

point_forces_built = [
    point_force(f[3], f[2], Num.(f[1]); frame = f[4])
    for f in force_inputs
]
body_torques_built = [
    body_torque(τ[2], Num.(τ[1]); frame = τ[3])
    for τ in torque_inputs
]

bodies = [make_body(i, chains[i];
            forces  = [f for f in point_forces_built if f.body_id == i],
            torques = [τ for τ in body_torques_built if τ.body_id == i])
          for i in 1:B]

sys = multibody_system(bodies)

inertial_vals = reduce(vcat,
    [vcat(mass_values[j], inertia_values[j], com_values[j]) for j in 1:B])
p_vals = vcat(config_values, inertial_vals)
p_value_dict = Dict(p_syms .=> p_vals)

dq = TDv(q)

# ======== REPLACED CELL — build the model, WITH added-mass kinetic energy ========
#
# ADDED MASS = kinetic energy of entrained fluid (Kirchhoff). Because it
# enters T, compute_M_h automatically emits BOTH the added-mass block in
# M(q) AND the added-mass Coriolis terms in h(q,q̇) — no extra machinery,
# and the 45-class → 10-class switched ablation now covers fluid coupling.
kin_auv = build_kinematics(sys, q, P)
KE_auv  = compute_KE(sys, P, kin_auv)

Xud = cfg(:Xud); Zwd = cfg(:Zwd); Mqd = cfg(:Mqd); Nrd = cfg(:Nrd)
ωb_hull = R_hull' * Omega(T_hull)         # body-frame angular velocity

T_fluid = (1//2)*(-Xud)*vo[1]^2 + (1//2)*(-Zwd)*vo[3]^2 +
          (1//2)*(-Mqd)*ωb_hull[2]^2 + (1//2)*(-Nrd)*ωb_hull[3]^2

Q_auv = compute_Q_total(sys, q, kin_auv)
M_auv, h_auv = compute_M_h(KE_auv.KE_total + T_fluid, q)

model = SymbolicModel(sys, P, q, TD.(q), vcat(q, TD.(q)), p_syms,
                      kin_auv, KE_auv, Q_auv, M_auv, h_auv)

# ======== BASELINE initial state (the run itself is in test/golden.jl) ========
#
# u = [x, z, θ, ψ, ẋ, ż, θ̇, ψ̇]   (8 states)
u0 = [0.0, 0.0, 0.05, 0.0,   1.0, 0.0, 0.0, 0.0]
u_test = u0                       # keeps the M-check cell working

# ======== guarded RHS + parameter index lookup (replaces hardcoded p[15]/p[16]) ========
dyn_a! = compile_ode(model)
function dyn!(du, u, p, tnow)
    # guard: |pitch| > 0.8 rad or non-finite → freeze (fin/hull model exits validity)
    if abs(u[3]) > 0.8 || !all(isfinite, u)
        du .= u .* 0
    else
        dyn_a!(du, u, p, tnow)
    end
end

pidx(s) = findfirst(x -> isequal(x, Symbolics.variable(s)), p_syms)
idx_T  = pidx(:T_thr)
idx_δ1 = pidx(:δ1)
idx_δ2 = pidx(:δ2)
idx_δ3 = pidx(:δ3)

# ======== B(q,q̇) — thruster/fin allocation matrix, EXTRACTED not written ========
#
# Q_total is linear in [T_thr, δ1, δ2, δ3] (thrust linear; fin lift ∝ CLα·δ),
# so ∂Q/∂u_act is the exact allocation matrix — state-dependent through qdyn:
# fin authority collapses quadratically as the vehicle slows.
u_act = Num[T_thr_s, δ1_s, δ2_s, δ3_s]
B_sym = jac(model.Q, u_act)

B_fun = build_function(statify.(B_sym), states_dync, model.p_syms;
                       expression = Val(false), cse = true)[1]

# ======== h CLASSIFICATION — numeric polarization, now eltype-generic ========
n_q = length(model.q)

h_fun_cls = build_function(statify.(model.h), states_dync, model.p_syms;
                           expression = Val(false), cse = true)[1]

clean(v) = replace(string(v), "(t)" => "")
snames   = clean.(model.q)

pair_list = [(i, j) for i in 1:n_q for j in i:n_q]
labels    = [i == j ? "d$(snames[i])^2" : "d$(snames[i])*d$(snames[j])"
             for (i, j) in pair_list]
K = length(pair_list)

function h_classes(u, p)
    T = promote_type(eltype(u), eltype(p))          # ← Float64 for sim, Num for proof
    qpos = u[1:n_q]; qdot = u[n_q+1:2n_q]
    ei = zeros(T, 2n_q); ei[1:n_q] .= qpos
    cdiag = Vector{Vector{T}}(undef, n_q)
    for i in 1:n_q
        ei[n_q+1:2n_q] .= 0; ei[n_q+i] = 1
        cdiag[i] = vec(h_fun_cls(ei, p))
    end
    parts = Vector{Vector{T}}(undef, K)
    for (k, (i, j)) in enumerate(pair_list)
        if i == j
            parts[k] = cdiag[i] .* qdot[i]^2
        else
            ei[n_q+1:2n_q] .= 0; ei[n_q+i] = 1; ei[n_q+j] = 1
            cij = vec(h_fun_cls(ei, p)) .- cdiag[i] .- cdiag[j]
            parts[k] = cij .* (qdot[i] * qdot[j])
        end
    end
    return parts
end

u_sym = Num[model.q...; model.qdot...]

# ======== SWITCHED DYNAMICS — switches in p, pitch guard (was roll guard) ========
M_fun_sw = build_function(statify.(model.M), states_dync, model.p_syms;
                          expression = Val(false), cse = true)[1]
Q_fun_sw = build_function(statify.(model.Q), states_dync, model.p_syms;
                          expression = Val(false), cse = true)[1]

n_p = length(model.p_syms)

function dynamics_sw!(du, u, p, tnow)
    @assert length(p) == n_p + K "p must be make_p(...) = [physical params; K switches]"
    # ── pitch guard: freeze past |θ| = 0.8 rad ──
    if abs(u[3]) > 0.8 || !all(isfinite, u)
        du .= u .* 0
        return nothing
    end
    pb = @view p[1:n_p]
    s  = @view p[n_p+1:n_p+K]
    parts = h_classes(u, pb)
    h_now = zeros(n_q)
    for k in 1:K
        s[k] != 0.0 && (h_now .+= s[k] .* parts[k])
    end
    rhs = vec(Q_fun_sw(u, pb)) .- h_now
    du[1:n_q]      .= u[n_q+1:2n_q]
    du[n_q+1:2n_q] .= M_fun_sw(u, pb) \ rhs
    return nothing
end

function make_p(switches::Dict = Dict(); base = p_vals)
    @assert length(base) == n_p "base params length ≠ n_p"
    s = ones(K)
    for (lab, v) in switches
        idx = findfirst(==(lab), labels)
        idx === nothing && error("unknown class '$lab' — check `labels`")
        s[idx] = float(v)
    end
    return vcat(base, s)
end

function show_switches(p)
    s   = p[n_p+1:end]
    off = [labels[k] for k in 1:K if s[k] == 0.0]
    sc  = [(labels[k], s[k]) for k in 1:K if !(s[k] in (0.0, 1.0))]
    println(K - length(off) - length(sc), " ON, ", length(off), " OFF",
            isempty(off) ? "" : ("  → OFF: " * join(off, ", ")))
    isempty(sc) || println("scaled: ", sc)
end

# base kwarg lets you sweep T_thr/δ through idx_T/idx_δ without touching the model
run_switched(u0s, tspan; switches = Dict(), base = p_vals,
             solver = AutoTsit5(Rosenbrock23(autodiff = AutoFiniteDiff())),
             reltol = 1e-6, abstol = 1e-6) =
    solve(ODEProblem(dynamics_sw!, u0s, tspan, make_p(switches; base = base)),
          solver; reltol, abstol, saveat = 0.01, maxiters = 5_000_000)

# ======== CLASS CONTRIBUTION TABLE ========
function class_table(solx; p = p_vals, nsamp = 200)
    ts   = range(solx.t[1], solx.t[end]; length = nsamp)
    maxc = zeros(K); rmsc = zeros(K)
    for τ in ts
        parts = h_classes(solx(τ), p)
        for k in 1:K
            v = norm(parts[k])
            maxc[k]  = max(maxc[k], v)
            rmsc[k] += v^2
        end
    end
    rmsc  = sqrt.(rmsc ./ nsamp)
    order = sortperm(maxc; rev = true)
    println(rpad("class", 18), rpad("max ‖h_k‖", 14), "rms ‖h_k‖")
    for k in order[1:min(15, K)]
        println(rpad(labels[k], 18),
                rpad(string(round(maxc[k], sigdigits = 4)), 14),
                round(rmsc[k], sigdigits = 4))
    end
    return (labels = labels[order], max = maxc[order], rms = rmsc[order])
end

# ============================================================================
# Not in the notebook: one function that hands the built model to other code
# (CLAUDE_AUV_PJT.md, Step 0). Nothing is rebuilt here; it only collects.
# ============================================================================
auv_model() = (; model, p_vals, pidx, idx_T, idx_δ1, idx_δ2, idx_δ3,
                 dyn!, B_fun, K, labels)

end # module AUVModel

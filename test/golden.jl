# Golden regression tests: CLAUDE_AUV_PJT.md Section 3.
# Each expected number is an output printed by the original notebook.
# If any of these fail after a refactor: STOP and report, do not adjust.
#
# Run from the repo root:  julia --project=. test/golden.jl
using Test
using LinearAlgebra
using Random
using Symbolics
using OrdinaryDiffEq
using ADTypes: AutoFiniteDiff

include(joinpath(@__DIR__, "..", "src", "auv_model.jl"))
const A = AUVModel

const STIFF_SOLVER = AutoTsit5(Rosenbrock23(autodiff = AutoFiniteDiff()))
solve_auv(u0, tspan, p) = solve(ODEProblem(A.dyn!, u0, tspan, p), STIFF_SOLVER;
                                reltol = 1e-6, abstol = 1e-6, saveat = 0.02,
                                maxiters = 5_000_000)

# The notebook prints 4 significant digits, so compare to half of the last digit
# plus a little room for solver-version differences.
close4(value, expected) = isapprox(value, expected; rtol = 2e-3)

@testset "AUV golden numbers (Section 3)" begin

    @testset "sizes and parameter vector" begin
        @test size(A.model.M) == (4, 4)
        @test length(A.model.h) == 4
        @test length(A.p_syms) == length(A.p_vals) == 28
        @test (A.idx_T, A.idx_δ1, A.idx_δ2, A.idx_δ3) == (1, 2, 3, 4)
    end

    @testset "statified M_fun vs substituted M" begin
        M_fun_chk = build_function(A.statify.(A.model.M), A.states_dync, A.model.p_syms;
                                   expression = Val(false), cse = true)[1]
        M_stat   = A.statify.(A.model.M)
        num_dict = Dict(vcat(A.states_dync .=> A.u_test, A.p_syms .=> A.p_vals))
        M_substituted = Symbolics.value.(A.subs(num_dict).(M_stat))
        err = maximum(abs.(Float64.(M_substituted) .- M_fun_chk(A.u_test, A.p_vals)))
        println("max |difference| = ", err)
        @test err < 1e-12
    end

    @testset "baseline: T = 8 N, fins centred, 30 s" begin
        sol = A.run_simulation(A.model, A.u0, (0.0, 30.0); p = A.p_vals,
                               solver = STIFF_SOLVER, reltol = 1e-6, abstol = 1e-6)
        uf = sol.u[end]
        println("retcode = ", sol.retcode, ", steps = ", length(sol.t))
        println("x = ", uf[1], "  z = ", uf[2], "  θ = ", uf[3],
                "  dx = ", uf[5], "  dz = ", uf[6])
        @test sol.retcode == ReturnCode.Success
        @test close4(uf[1], 25.01)
        @test close4(uf[2], -0.3997)
        @test close4(uf[3], 0.09109)
        @test close4(uf[5], 0.8289)
        @test close4(uf[6], -0.01381)
        @test abs(uf[4]) < 1e-12        # yaw is round-off only (Section 4, issue 3)
    end

    @testset "thrust sweep from rest, 40 s" begin
        for (thrust_N, speed_m_s) in [(2.0, 0.362), (5.0, 0.633), (10.0, 0.939), (15.0, 1.173)]
            psp = copy(A.p_vals)
            psp[A.idx_T] = thrust_N
            s = solve_auv(zeros(8), (0.0, 40.0), psp)
            println("T = ", thrust_N, "  u_end = ", round(s.u[end][5], digits = 3), " m/s")
            @test s.retcode == ReturnCode.Success
            @test round(s.u[end][5], digits = 3) ≈ speed_m_s atol = 1.5e-3
        end
    end

    @testset "fin steps, T = 8 N, start at 1 m/s" begin
        u0f = zeros(8); u0f[5] = 1.0

        p_pitch = copy(A.p_vals); p_pitch[A.idx_T] = 8.0
        p_pitch[A.idx_δ2] = 0.20; p_pitch[A.idx_δ3] = -0.20
        s_pitch = solve_auv(u0f, (0.0, 25.0), p_pitch)
        println("pitch step: θ_end = ", s_pitch.u[end][3], "  z_end = ", s_pitch.u[end][2])
        @test isapprox(s_pitch.u[end][3], -0.14; atol = 0.005)   # nose up
        @test s_pitch.u[end][2] > 1.0                            # z rises
        @test abs(s_pitch.u[end][4]) < 1e-6                      # almost no yaw

        p_yaw = copy(A.p_vals); p_yaw[A.idx_T] = 8.0
        p_yaw[A.idx_δ1] = 0.20
        s_yaw = solve_auv(u0f, (0.0, 25.0), p_yaw)
        println("yaw step:   ψ_end = ", s_yaw.u[end][4])
        @test isapprox(s_yaw.u[end][4], -0.10; atol = 0.005)
    end

    @testset "allocation matrix B at u0" begin
        B_num = A.B_fun(A.u_test, A.p_vals)
        display(round.(B_num, sigdigits = 3))
        @test rank(B_num, rtol = 1e-8) == 3
        expected = [ 0.999  0.0  -0.25  0.25
                    -0.05   0.0  -5.0   5.0
                     0.0    0.0  -2.1   2.1
                     0.0   -2.4   1.24  1.24]
        @test isapprox(B_num, expected; atol = 0.02)
    end

    @testset "h classes: K = 10, polarization exact to round-off" begin
        @test A.K == 10
        @test length(A.labels) == 10
        @test length(A.make_p()) == A.n_p + A.K

        rng = MersenneTwister(1)
        function rand_state()
            u = zeros(2 * A.n_q)
            u[3:4] .= 0.1 .* randn(rng, 2)     # perturb pitch / yaw
            u[5]    = 0.5 + 1.5 * rand(rng)    # forward speed > 0
            u[6:8] .= 0.3 .* randn(rng, 3)     # random velocities
            return u
        end
        dyn_base! = A.compile_ode(A.model)
        du_a = zeros(8); du_b = zeros(8)
        for trial in 1:3
            u = rand_state()
            h_full = vec(A.h_fun_cls(u, A.p_vals))

            # h is homogeneous quadratic in q̇
            u2 = copy(u); u2[A.n_q+1:2*A.n_q] .*= 2.0
            @test maximum(abs.(vec(A.h_fun_cls(u2, A.p_vals)) .- 4 .* h_full)) < 1e-12

            # Σ parts == h (numerically verified to round-off, NOT a symbolic proof)
            e = maximum(abs.(reduce(+, A.h_classes(u, A.p_vals)) .- h_full))
            println("|Σ parts - h|∞ = ", e)
            @test e < 1e-12

            # dynamics_sw! with all switches ON == compile_ode
            dyn_base!(du_a, u, A.p_vals, 0.0)
            A.dynamics_sw!(du_b, u, A.make_p(), 0.0)
            e = maximum(abs.(du_a .- du_b))
            println("|du_sw - du_base|∞ = ", e)
            @test e < 1e-12
        end
    end

    @testset "switched ablation: dxh^2 OFF drops the pitch trim ~65%" begin
        sol_on  = A.run_switched(A.u0, (0.0, 20.0))
        tbl     = A.class_table(sol_on)
        sol_off = A.run_switched(A.u0, (0.0, 20.0); switches = Dict(tbl.labels[1] => 0))
        θ_on, θ_off = sol_on.u[end][3], sol_off.u[end][3]
        println("θ_end all ON = ", θ_on, "   dxh^2 OFF = ", θ_off,
                "   drop = ", round(100 * (1 - θ_off / θ_on), digits = 1), " %")
        @test tbl.labels[1] == "dxh^2"
        @test isapprox(θ_on, 0.0911; atol = 0.001)
        @test isapprox(θ_off, 0.0323; atol = 0.001)
    end
end

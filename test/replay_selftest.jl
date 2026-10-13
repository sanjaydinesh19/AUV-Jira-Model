# Self-test of analysis/replay.jl using a SYNTHETIC log.
#
# There is no real pool log yet. To check that the replay code itself is
# correct we let the closed-loop simulation write a log in the Pi's CSV format,
# then replay it. This proves the plumbing (file format, zero-order hold,
# time stamps), NOT that the model matches the vehicle.
#
#   1. Same parameters for "vehicle" and model  -> error must be ~0.
#   2. "Vehicle" with a 20% lower fin lift slope -> replay must show an error,
#      i.e. the tool can see a wrong parameter.
#
# Run from the repo root:  julia --project=. test/replay_selftest.jl
using Test
using Printf

include(joinpath(@__DIR__, "..", "analysis", "replay.jl"))

function write_synthetic_log(path, sim, thrust_N)
    open(path, "w") do io
        println(io, "t,thrust_cmd,d1,d2,d3,pitch_meas,yaw_meas,pitch_rate,yaw_rate,roll_meas,mode")
        for k in eachindex(sim.t)
            @printf(io, "%.3f,%.1f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.8f,%.1f,SIM\n",
                    sim.t[k], thrust_N, sim.fins[k, 1], sim.fins[k, 2], sim.fins[k, 3],
                    sim.state[k, 3], sim.state[k, 4], sim.state[k, 7], sim.state[k, 8], 0.0)
        end
    end
end

thrust_N = 8.0
pitch_ref(t) = t < 5.0 ? 0.0 : deg2rad(-5.0)
yaw_ref(t)   = t < 10.0 ? 0.0 : deg2rad(8.0)

@testset "replay self-test (synthetic logs)" begin
    mktempdir() do dir
        @testset "same parameters: replay reproduces the log" begin
            sim = simulate_closed_loop(; thrust_N, t_end_s = 20.0, pitch_ref, yaw_ref)
            path = joinpath(dir, "same.csv")
            write_synthetic_log(path, sim, thrust_N)
            run_log = read_log(path)
            pred = replay(run_log)
            θ_rms_deg = rad2deg(rms(pred.θ_pred .- run_log.θ))
            ψ_rms_deg = rad2deg(rms(pred.ψ_pred .- run_log.ψ))
            @printf("same parameters:   RMS θ = %.5f°   RMS ψ = %.5f°\n", θ_rms_deg, ψ_rms_deg)
            @test θ_rms_deg < 0.01
            @test ψ_rms_deg < 0.01
        end

        @testset "wrong CLa: replay shows the mismatch" begin
            p_vehicle = copy(A.p_vals)
            p_vehicle[A.pidx(:CLa)] *= 0.8
            sim = simulate_closed_loop(; thrust_N, t_end_s = 20.0, pitch_ref, yaw_ref,
                                       p_base = p_vehicle)
            path = joinpath(dir, "wrong_cla.csv")
            write_synthetic_log(path, sim, thrust_N)
            run_log = read_log(path)
            pred = replay(run_log)                 # model still uses the nominal CLa
            θ_rms_deg = rad2deg(rms(pred.θ_pred .- run_log.θ))
            @printf("CLa 20%% too high:  RMS θ = %.3f°\n", θ_rms_deg)
            @test θ_rms_deg > 0.2
        end
    end
end

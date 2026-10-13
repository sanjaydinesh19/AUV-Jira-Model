# PID with the two features Section 6 Step 1 asks for:
#   - the D term uses the measured RATE (gyro), never a differentiated angle,
#     so a step in the reference does not kick the fins;
#   - the integral is clamped and can be frozen while the fins are saturated
#     (anti-windup), so it does not keep growing when the fins cannot do more.

mutable struct PID
    kp::Float64
    ki::Float64
    kd::Float64
    integral_limit::Float64   # clamp on ∫error dt
    integral::Float64         # ∫error dt, the controller's only memory
end

PID(kp, ki, kd, integral_limit) = PID(kp, ki, kd, integral_limit, 0.0)

# Wrap an angle to (-π, π]. Used for the heading error so that a reference of
# +179° and a heading of -179° give a 2° error, not 358°.
function wrap_pi(angle_rad)
    wrapped = mod(angle_rad + π, 2π)
    wrapped <= 0 && (wrapped += 2π)
    return wrapped - π
end

# One controller step. `integrate = false` freezes the integral (anti-windup).
function pid_update!(pid::PID, error, rate, dt_s; integrate = true)
    if integrate
        pid.integral = clamp(pid.integral + error * dt_s,
                             -pid.integral_limit, pid.integral_limit)
    end
    return pid.kp * error + pid.ki * pid.integral - pid.kd * rate
end

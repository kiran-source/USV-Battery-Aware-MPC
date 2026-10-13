function [T_pid, e_int_new, e_curr] = pid_controller(u_meas, u_ref, e_int_prev, e_prev, dt, params)
    % PID_CONTROLLER Discrete PID speed controller with anti-windup and derivative filtering
    %
    % Inputs:
    %   u_meas     : Measured USV speed [m/s]
    %   u_ref      : Desired speed [m/s]
    %   e_int_prev : Accumulated integral error from previous step
    %   e_prev     : Error from previous step
    %   dt         : Time step [s]
    %   params     : Parameter struct from parameters()
    %
    % Outputs:
    %   T_pid      : Commanded thrust [N], clamped to [T_min, T_max]
    %   e_int_new  : Updated integral error
    %   e_curr     : Current tracking error

    Kp = params.pid.Kp;
    Ki = params.pid.Ki;
    Kd = params.pid.Kd;
    N  = params.pid.N;
    T_min = params.usv.T_min;
    T_max = params.usv.T_max;

    e_curr = u_ref - u_meas;

    % Proportional term
    P_term = Kp * e_curr;

    % Derivative term with first-order filter
    % d/dt e approx filtered: Kd * N * (e_curr - e_prev) / (1 + N * dt)
    D_term = (Kd * N * (e_curr - e_prev)) / (1 + N * dt);

    % Tentative integral term
    e_int_tentative = e_int_prev + e_curr * dt;
    I_term = Ki * e_int_tentative;

    % Raw thrust command
    T_raw = P_term + I_term + D_term;

    % Saturate thrust
    T_pid = max(min(T_raw, T_max), T_min);

    % Anti-windup clamping on integral state
    if (T_raw > T_max && e_curr > 0) || (T_raw < T_min && e_curr < 0)
        % Clamp integrator (freeze) to prevent integrator windup
        e_int_new = e_int_prev;
    else
        e_int_new = e_int_tentative;
    end
end

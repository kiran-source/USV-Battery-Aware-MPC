function [T_cmd, N_cmd, state_pid] = PID_controller(u_ref, psi_ref, u_meas, psi_meas, r_meas, dt, state_pid, p)
    % PID_CONTROLLER Dual-Loop Speed (Surge) and Heading (Yaw) PID Baseline
    %
    % Inputs:
    %   u_ref     - Target surge velocity [m/s]
    %   psi_ref   - Target heading angle [rad]
    %   u_meas    - Measured surge velocity [m/s]
    %   psi_meas  - Measured heading angle [rad]
    %   r_meas    - Measured yaw rate [rad/s]
    %   dt        - Timestep [s]
    %   state_pid - Struct containing integrators and previous errors
    %   p         - Full parameter struct
    %
    % Outputs:
    %   T_cmd     - Total surge thrust command [N]
    %   N_cmd     - Differential yaw moment command [N*m]
    %   state_pid - Updated integrator states
    
    if isempty(state_pid)
        state_pid.int_eu   = 0.0;
        state_pid.prev_eu  = 0.0;
        state_pid.int_epsi = 0.0;
        state_pid.prev_epsi= 0.0;
    end
    
    % --- SPEED LOOP GAINS ---
    Kp_u = 140.0;
    Ki_u = 18.0;
    Kd_u = 12.0;
    
    % --- HEADING LOOP GAINS ---
    Kp_psi = 65.0;
    Ki_psi = 4.0;
    Kd_psi = 35.0; % High derivative damping on yaw rate
    
    %% 1. SPEED ERROR & PID CONTROL
    e_u = u_ref - u_meas;
    state_pid.int_eu = state_pid.int_eu + e_u * dt;
    % Anti-windup clamping on speed integrator
    state_pid.int_eu = min(max(state_pid.int_eu, -5.0), 5.0);
    
    der_eu = (e_u - state_pid.prev_eu) / dt;
    state_pid.prev_eu = e_u;
    
    T_raw = Kp_u * e_u + Ki_u * state_pid.int_eu + Kd_u * der_eu;
    T_cmd = min(max(T_raw, p.usv.prop.T_min_tot), p.usv.prop.T_max_tot);
    
    %% 2. HEADING ERROR & PID CONTROL
    e_psi = wrapToPi(psi_ref - psi_meas);
    state_pid.int_epsi = state_pid.int_epsi + e_psi * dt;
    % Anti-windup clamping on heading integrator
    state_pid.int_epsi = min(max(state_pid.int_epsi, -1.0), 1.0);
    
    % Use measured yaw rate for derivative action
    N_raw = Kp_psi * e_psi + Ki_psi * state_pid.int_epsi - Kd_psi * r_meas;
    N_cmd = min(max(N_raw, -p.usv.prop.N_max), p.usv.prop.N_max);
end

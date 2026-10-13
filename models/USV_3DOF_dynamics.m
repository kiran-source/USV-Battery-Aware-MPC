function [eta_dot, nu_dot, M, C, D, tau_env] = USV_3DOF_dynamics(eta, nu, tau_ctrl, t, p_usv, p_env)
    % USV_3DOF_DYNAMICS Complete 3-DOF Marine Vessel Equations of Motion
    %
    % State Vectors:
    %   eta = [x; y; psi] - Global NED position & heading [m, m, rad]
    %   nu  = [u; v; r]   - Body-fixed velocity vector [m/s, m/s, rad/s]
    %
    % Inputs:
    %   tau_ctrl - Control force/moment vector [T; 0; N] [N, N, N*m]
    %   t        - Time [s]
    %   p_usv    - USV parameters
    %   p_env    - Environmental parameters
    %
    % Outputs:
    %   eta_dot  - Global velocity vector [x_dot; y_dot; psi_dot]
    %   nu_dot   - Body acceleration vector [u_dot; v_dot; r_dot]
    
    psi = eta(3);
    
    %% 1. KINEMATIC ROTATION MATRIX
    R = [
        cos(psi),  -sin(psi),  0;
        sin(psi),   cos(psi),  0;
        0,          0,         1
    ];
    
    eta_dot = R * nu;
    
    %% 2. ENVIRONMENTAL DISTURBANCES & RELATIVE VELOCITY
    [tau_env, nu_r, ~] = environmental_forces(eta, nu, t, p_env, p_usv);
    
    %% 3. HYDRODYNAMIC MATRICES & DAMPING FORCES
    [M, C, D, tau_D] = hydrodynamics(nu, nu_r, p_usv);
    
    %% 4. KINETIC ACCELERATION (M * nu_dot = tau_ctrl + tau_env - C*nu - tau_D)
    forces_net = tau_ctrl + tau_env - C * nu - tau_D;
    nu_dot = M \ forces_net;
end

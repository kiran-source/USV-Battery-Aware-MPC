function p_mpc = MPC_parameters()
    % MPC_PARAMETERS Configuration for 3-DOF Battery-Aware Model Predictive Controller
    
    % --- Horizons & Timing ---
    p_mpc.Ts = 0.25;      % Control execution sample time [s] (4 Hz)
    p_mpc.Np = 20;        % Prediction horizon steps (5.0 seconds look-ahead)
    p_mpc.Nc = 6;         % Control horizon steps (1.5 seconds)
    
    % --- Multi-Objective Optimization Weights ---
    p_mpc.weights.w_speed     = 100.0;  % Surge speed tracking error weight (u - u_ref)^2
    p_mpc.weights.w_cte       = 10.0;   % Cross-track position error weight (e_y)^2
    p_mpc.weights.w_psi       = 40.0;   % Heading angle tracking error weight (psi - psi_ref)^2
    p_mpc.weights.w_yaw_rate  = 5.0;    % Damping on excessive yaw rate (r)^2
    p_mpc.weights.w_power     = 0.08;   % Battery power consumption penalty (P_batt)
    p_mpc.weights.w_rate_T    = 0.05;   % Thruster surge slew rate penalty (delta_T)^2
    p_mpc.weights.w_rate_N    = 0.10;   % Differential yaw moment slew rate (delta_N)^2
    p_mpc.weights.w_soc_slack = 5000.0; % Heavy penalty on SOC reserve violations
    
    % --- Physical Constraints ---
    p_mpc.bounds.T_min       = 0.0;     % Minimum total forward thrust [N]
    p_mpc.bounds.T_max       = 150.0;   % Maximum total forward thrust [N]
    p_mpc.bounds.N_max       = 45.0;    % Maximum differential yaw moment [N*m]
    p_mpc.bounds.delta_T_max = 30.0;    % Max thrust change per timestep (Ts * 120 N/s) [N]
    p_mpc.bounds.delta_N_max = 12.0;    % Max yaw moment change per timestep [N*m]
    p_mpc.bounds.u_min       = 0.0;     % Surge velocity minimum [m/s]
    p_mpc.bounds.u_max       = 2.8;     % Surge velocity maximum [m/s]
    p_mpc.bounds.r_max       = deg2rad(35.0); % Yaw rate maximum [rad/s]
    p_mpc.bounds.cte_max     = 10.0;    % Maximum allowable cross-track error [m]
    
    % --- Solver Configuration ---
    p_mpc.solver.max_iter    = 100;     % Max QP active-set iterations
    p_mpc.solver.tol         = 1e-4;    % Gradient convergence tolerance
end

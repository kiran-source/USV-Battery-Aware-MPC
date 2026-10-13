function p_usv = USV_parameters()
    % USV_PARAMETERS Centralized physical and hydrodynamic parameters for 3-DOF USV
    % Model: 3-DOF Horizontal Plane (Surge, Sway, Yaw)
    % Convention: SNAME (1950) / Fossen (2011)
    
    % --- Rigid-Body Inertial Properties ---
    p_usv.m   = 180.0;    % Vessel dry mass [kg]
    p_usv.Iz  = 75.0;     % Yaw moment of inertia about CG [kg*m^2]
    p_usv.xg  = 0.05;     % Longitudinal CG offset from body origin [m]
    p_usv.L   = 2.2;      % Hull overall length [m]
    p_usv.B   = 0.8;      % Hull beam (width) [m]
    
    % --- Hydrodynamic Added Mass (Inviscid Fluid Effects) ---
    p_usv.X_udot = -20.0;  % Added mass in surge [kg]
    p_usv.Y_vdot = -45.0;  % Added mass in sway [kg]
    p_usv.Y_rdot = -1.5;   % Added inertia cross-term [kg*m]
    p_usv.N_vdot = -1.5;   % Added inertia cross-term [kg*m]
    p_usv.N_rdot = -22.0;  % Added yaw moment of inertia [kg*m^2]
    
    % --- Hydrodynamic Damping Coefficients ---
    % Linear damping (viscous skin friction / low speed)
    p_usv.Xu = -25.0;     % Surge linear drag [N/(m/s)]
    p_usv.Yv = -60.0;     % Sway linear drag [N/(m/s)]
    p_usv.Yr = -5.0;      % Sway-yaw cross drag [N/(rad/s)]
    p_usv.Nv = -5.0;      % Yaw-sway cross drag [N*m/(m/s)]
    p_usv.Nr = -30.0;     % Yaw linear drag [N*m/(rad/s)]
    
    % Nonlinear / Quadratic damping (form drag / cross-flow drag)
    p_usv.Xuu = -35.0;    % Surge quadratic drag [N/(m/s)^2]
    p_usv.Yvv = -120.0;   % Sway quadratic drag [N/(m/s)^2]
    p_usv.Yrr = -10.0;    % Sway-yaw quadratic drag [N/(rad/s)^2]
    p_usv.Nvv = -10.0;    % Yaw-sway quadratic drag [N*m/(m/s)^2]
    p_usv.Nrr = -45.0;    % Yaw quadratic drag [N*m/(rad/s)^2]
    
    % --- Propulsion Configuration (Differential Dual Thrusters) ---
    p_usv.prop.B_prop    = 0.60;   % Thruster separation baseline (beam between motors) [m]
    p_usv.prop.arm       = p_usv.prop.B_prop / 2; % Lever arm = 0.30 m
    p_usv.prop.T_max_ind = 150.0;  % Max thrust per motor [N] (Total max surge thrust = 300 N)
    p_usv.prop.T_min_ind = -50.0;  % Reverse thrust per motor [N]
    p_usv.prop.T_max_tot = 2 * p_usv.prop.T_max_ind; % 300 N
    p_usv.prop.T_min_tot = 0.0;    % Forward cruise minimum [N]
    p_usv.prop.N_max     = (p_usv.prop.T_max_ind - p_usv.prop.T_min_ind) * p_usv.prop.arm; % Max yaw moment [N*m]
    p_usv.prop.rate_max  = 120.0;  % Max thrust slew rate [N/s]
    
    % Motor & Drive Efficiency
    p_usv.prop.eta_motor = 0.75;   % Electromechanical efficiency (motor + prop + ESC)
    p_usv.prop.k_loss    = 0.015;  % Static / bollard electrical loss coefficient [W/N^2]
    
    % --- Operational Constraints ---
    p_usv.u_max = 3.0;             % Max surge velocity [m/s] (~5.8 knots)
    p_usv.v_max = 1.0;             % Max sway velocity [m/s]
    p_usv.r_max = deg2rad(45.0);   % Max yaw rate [rad/s] (45 deg/s)
end

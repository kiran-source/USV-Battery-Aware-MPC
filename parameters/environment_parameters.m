function p_env = environment_parameters()
    % ENVIRONMENT_PARAMETERS Configuration for Marine Environmental Disturbances
    
    % --- Water Current Model ---
    p_env.current.enabled   = true;
    p_env.current.speed     = 0.35;         % Current flow speed [m/s] (~0.7 knots)
    p_env.current.direction = deg2rad(180); % Current flow direction [rad] (180 deg = heading West / opposing eastward transit)
    p_env.current.type      = 'constant';   % 'none', 'constant', 'opposing', 'assisting', 'cross'
    
    % --- Wind Disturbance Model ---
    p_env.wind.enabled     = true;
    p_env.wind.speed       = 4.5;          % True wind speed at 10m height [m/s] (~8.7 knots)
    p_env.wind.direction   = deg2rad(225); % Direction wind is blowing FROM [rad] (South-West)
    p_env.wind.rho_air     = 1.225;        % Air density [kg/m^3]
    p_env.wind.A_frontal   = 0.45;         % Frontal projected windage area [m^2]
    p_env.wind.A_lateral   = 0.95;         % Lateral projected windage area [m^2]
    p_env.wind.Cd_frontal  = 0.70;         % Frontal aerodynamic drag coefficient
    p_env.wind.Cd_lateral  = 0.90;         % Lateral aerodynamic drag coefficient
    
    % --- Wave Disturbance Model (Reduced-Order 1st/2nd Order Wave Action) ---
    p_env.wave.enabled     = true;
    p_env.wave.amplitude   = 15.0;         % Peak wave disturbance force amplitude [N]
    p_env.wave.moment_amp  = 5.0;          % Peak wave disturbance moment amplitude [N*m]
    p_env.wave.frequency   = 0.80;         % Dominant wave encounter frequency [rad/s]
    p_env.wave.direction   = deg2rad(190); % Mean wave encounter direction [rad]
    p_env.wave.noise_std   = 3.0;          % Bounded stochastic disturbance noise standard deviation [N]
end

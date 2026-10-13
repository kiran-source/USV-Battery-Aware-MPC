function [tau_env, nu_r, V_c_body] = environmental_forces(eta, nu, t, p_env, p_usv)
    % ENVIRONMENTAL_FORCES Computes Marine Environmental Disturbances (Current, Wind, Waves)
    %
    % Inputs:
    %   eta   = [x; y; psi] - USV position & heading in NED frame [m, m, rad]
    %   nu    = [u; v; r]   - USV body-fixed velocity [m/s, m/s, rad/s]
    %   t                   - Current simulation time [s]
    %   p_env               - Environmental parameters struct
    %   p_usv               - USV parameters struct
    %
    % Outputs:
    %   tau_env   - Direct environmental disturbance force/moment [N; N; N*m]
    %   nu_r      - Relative velocity vector [u_r; v_r; r] in body frame [m/s]
    %   V_c_body  - Current velocity expressed in body frame [u_c; v_c] [m/s]
    
    psi = eta(3);
    u   = nu(1);
    v   = nu(2);
    r   = nu(3);
    
    %% 1. WATER CURRENT (Relative Flow Model)
    if p_env.current.enabled
        Vc_mag = p_env.current.speed;
        beta_c = p_env.current.direction;
        
        % Current in NED frame
        Vc_x = Vc_mag * cos(beta_c);
        Vc_y = Vc_mag * sin(beta_c);
        
        % Transform current to Body Frame (Rotation matrix transpose R(psi)^T)
        u_c =  Vc_x * cos(psi) + Vc_y * sin(psi);
        v_c = -Vc_x * sin(psi) + Vc_y * cos(psi);
    else
        u_c = 0.0;
        v_c = 0.0;
    end
    
    V_c_body = [u_c; v_c];
    
    % Relative velocity to surrounding water: nu_r = nu - [u_c; v_c; 0]
    u_r = u - u_c;
    v_r = v - v_c;
    nu_r = [u_r; v_r; r];
    
    %% 2. WIND DISTURBANCE FORCES
    tau_wind = zeros(3, 1);
    if p_env.wind.enabled
        Vw_mag = p_env.wind.speed;
        beta_w = p_env.wind.direction;
        
        % Wind velocity in NED (blowing towards beta_w + pi)
        Vw_x = -Vw_mag * cos(beta_w);
        Vw_y = -Vw_mag * sin(beta_w);
        
        % Relative wind velocity in body frame
        u_rw = u - (Vw_x * cos(psi) + Vw_y * sin(psi));
        v_rw = v - (-Vw_x * sin(psi) + Vw_y * cos(psi));
        
        V_rw = sqrt(u_rw^2 + v_rw^2);
        gamma_w = -atan2(v_rw, u_rw); % Apparent wind angle of attack
        
        q_wind = 0.5 * p_env.wind.rho_air * (V_rw^2);
        
        % Aerodynamic drag forces (Blendermann/Isherwood simplified model)
        Cx = -p_env.wind.Cd_frontal * cos(gamma_w);
        Cy =  p_env.wind.Cd_lateral * sin(gamma_w);
        Cn =  0.15 * p_env.wind.Cd_lateral * p_usv.L * sin(2 * gamma_w);
        
        tau_wind(1) = q_wind * p_env.wind.A_frontal * Cx;
        tau_wind(2) = q_wind * p_env.wind.A_lateral * Cy;
        tau_wind(3) = q_wind * p_env.wind.A_lateral * Cn;
    end
    
    %% 3. WAVE DISTURBANCE FORCES (Oscillatory 1st Order + Mean 2nd Order Drift)
    tau_wave = zeros(3, 1);
    if p_env.wave.enabled
        omega_e = p_env.wave.frequency;
        A_wave  = p_env.wave.amplitude;
        M_wave  = p_env.wave.moment_amp;
        
        % Angle of wave encounter
        beta_wave = p_env.wave.direction - psi;
        
        % Bounded oscillatory forces
        tau_wave(1) = A_wave * cos(beta_wave) * cos(omega_e * t) + 0.3 * A_wave * (cos(beta_wave)^2);
        tau_wave(2) = A_wave * sin(beta_wave) * sin(omega_e * t) + 0.3 * A_wave * (sin(beta_wave)^2);
        tau_wave(3) = M_wave * sin(beta_wave) * cos(omega_e * t + pi/4);
        
        % Bounded random sea clutter (deterministic seed based on time)
        pseudo_noise = sin(2.71 * t) * p_env.wave.noise_std;
        tau_wave(2) = tau_wave(2) + pseudo_noise;
    end
    
    %% TOTAL DIRECT ENVIRONMENTAL LOAD
    tau_env = tau_wind + tau_wave;
end

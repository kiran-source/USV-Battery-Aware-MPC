function [cost, E_est_Wh, t_est_s, dist_m] = route_cost(waypoints, u_cruise, p_env, p_usv, p_bat, weights)
    % ROUTE_COST Evaluates Energy, Time, Distance, and Multi-Objective Cost for a Candidate Path
    %
    % Inputs:
    %   waypoints - [N x 2] matrix of waypoint coordinates [x, y]
    %   u_cruise  - Assumed cruising speed along path [m/s]
    %   p_env     - Environmental disturbance parameters (current, wind)
    %   p_usv     - USV hydrodynamic parameters
    %   p_bat     - Battery parameters
    %   weights   - Multi-objective weights struct (w_distance, w_energy, w_time, w_risk)
    
    if nargin < 6
        weights.w_distance = 0.35;
        weights.w_energy   = 0.45;
        weights.w_time     = 0.15;
        weights.w_risk     = 0.05;
    end
    
    N_pts = size(waypoints, 1);
    if N_pts < 2
        cost = inf; E_est_Wh = 0; t_est_s = 0; dist_m = 0;
        return;
    end
    
    dist_m = 0;
    t_est_s = 0;
    E_est_J = 0;
    
    % Environmental current vector in NED
    if p_env.current.enabled
        Vc_x = p_env.current.speed * cos(p_env.current.direction);
        Vc_y = p_env.current.speed * sin(p_env.current.direction);
    else
        Vc_x = 0; Vc_y = 0;
    end
    
    for k = 1:(N_pts - 1)
        dx = waypoints(k+1, 1) - waypoints(k, 1);
        dy = waypoints(k+1, 2) - waypoints(k, 2);
        seg_dist = norm([dx, dy]);
        seg_psi  = atan2(dy, dx);
        
        % Current projected along heading
        u_c = Vc_x * cos(seg_psi) + Vc_y * sin(seg_psi);
        
        % Relative surge velocity to water
        u_r = max(u_cruise - u_c, 0.2);
        
        % Hydrodynamic drag & steady thrust
        T_ss = abs(p_usv.Xu) * u_r + abs(p_usv.Xuu) * (u_r^2);
        
        % Power draw
        P_prop = (T_ss * u_r) / p_usv.prop.eta_motor + p_usv.prop.k_loss * (T_ss^2);
        P_tot  = P_prop + p_bat.P_hotel;
        
        seg_time = seg_dist / max(u_cruise, 0.1);
        seg_energy = P_tot * seg_time;
        
        dist_m  = dist_m + seg_dist;
        t_est_s = t_est_s + seg_time;
        E_est_J = E_est_J + seg_energy;
    end
    
    E_est_Wh = E_est_J / 3600.0;
    
    % Normalization baselines
    d_ref = 1500.0; % [m]
    E_ref = 400.0;  % [Wh]
    t_ref = 800.0;  % [s]
    
    cost = weights.w_distance * (dist_m / d_ref) + ...
           weights.w_energy   * (E_est_Wh / E_ref) + ...
           weights.w_time     * (t_est_s / t_ref);
end

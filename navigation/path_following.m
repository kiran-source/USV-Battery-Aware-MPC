function [u_ref, psi_ref, e_y, e_psi] = path_following(eta, nu, waypoints, wp_idx, decision_mode, p)
    % PATH_FOLLOWING Aggregates LOS Guidance and Battery-Aware Speed Scheduling
    %
    % Inputs:
    %   eta           = [x; y; psi] - Current USV pose
    %   nu            = [u; v; r]   - Current USV velocities
    %   waypoints     - [N x 2] matrix of path waypoints
    %   wp_idx        - Active target waypoint index
    %   decision_mode - Mission mode (1=CRUISE, 2=ECONOMY, 3=DOCK, 4=RETURN, 5=EMERGENCY)
    %   p             - Parameter struct
    %
    % Outputs:
    %   u_ref   - Desired surge speed [m/s]
    %   psi_ref - Desired heading angle [rad]
    %   e_y     - Cross-track error [m]
    %   e_psi   - Heading error [rad]
    
    pos_xy = eta(1:2);
    psi = eta(3);
    
    N_wp = size(waypoints, 1);
    wp_idx = max(min(wp_idx, N_wp), 1);
    
    if wp_idx == 1
        pt_start = pos_xy;
        pt_end   = waypoints(1, :);
    else
        pt_start = waypoints(wp_idx - 1, :);
        pt_end   = waypoints(wp_idx, :);
    end
    
    %% 1. LOS GUIDANCE HEADING
    [psi_ref, e_y, ~, ~] = LOS_guidance(pos_xy, pt_start, pt_end, p.mission.guidance.Delta_LOS);
    e_psi = wrapToPi(psi - psi_ref);
    
    %% 2. BATTERY-AWARE SPEED SCHEDULING
    switch decision_mode
        case 1 % CRUISE (High speed)
            u_base = p.mission.speeds.u_nominal;
        case 2 % ECONOMY (Energy saving)
            u_base = p.mission.speeds.u_economy;
        case 3 % DIVERT_TO_DOCK
            u_base = p.mission.speeds.u_economy;
        case 4 % RETURN_TO_BASE
            u_base = p.mission.speeds.u_economy;
        otherwise % 5: EMERGENCY (Crawl)
            u_base = p.mission.speeds.u_low_power;
    end
    
    % Speed reduction on sharp turns (turn-coordination)
    % Cuts speed if heading error is large to prevent wide drift
    heading_turn_factor = max(cos(e_psi), 0.4);
    
    % Final arrival slowdown
    dist_to_final = norm(pos_xy - waypoints(end, :));
    if dist_to_final <= p.mission.guidance.R_capture_dest
        u_ref = 0.0; % Docking complete stop
    elseif dist_to_final <= 25.0
        u_ref = min(u_base * heading_turn_factor, p.mission.speeds.u_docking);
    else
        u_ref = u_base * heading_turn_factor;
    end
end

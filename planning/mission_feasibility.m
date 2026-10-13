function [feas_report, decision_mode] = mission_feasibility(pos_xy, SOC, P_batt, u, waypoints, wp_idx, p)
    % MISSION_FEASIBILITY Evaluates Reachability of Next WP, Destination, Dock, and Base
    %
    % Inputs:
    %   pos_xy    - [x, y] current USV position [m]
    %   SOC       - Current battery percentage [%]
    %   P_batt    - Current total electrical power [W]
    %   u         - Current surge speed [m/s]
    %   waypoints - Active path waypoints [N x 2]
    %   wp_idx    - Current active waypoint index
    %   p         - Parameters struct
    %
    % Outputs:
    %   feas_report   - Detailed struct of distances, required energies, and reachability flags
    %   decision_mode - 1=CRUISE, 2=ECONOMY, 3=DIVERT_TO_DOCK, 4=RETURN_TO_BASE, 5=EMERGENCY
    
    E_cap     = p.battery.capacity_J;
    margin    = p.mission.safety_margin;
    SOC_res   = p.battery.soc_reserve_pct;
    
    %% 1. ENERGY & DISCHARGE RATE CALCULATION
    % Specific energy rate [J/m] (energy needed per meter)
    e_rate = max(P_batt / max(abs(u), 0.1), 30.0); % Bounded by minimum 30 J/m
    
    E_avail  = (SOC / 100.0) * E_cap; % Total energy in pack [J]
    E_res_J  = (SOC_res / 100.0) * E_cap; % Reserve energy [J]
    E_usable = max(E_avail - E_res_J, 0.0); % Usable energy before hitting reserve [J]
    
    %% 2. DISTANCE CALCULATIONS
    % Distance along remaining path to destination
    N_wp = size(waypoints, 1);
    wp_idx = max(min(wp_idx, N_wp), 1);
    
    d_dest = norm(pos_xy - waypoints(wp_idx, :));
    for k = wp_idx:(N_wp - 1)
        d_dest = d_dest + norm(waypoints(k+1, :) - waypoints(k, :));
    end
    
    base_xy = [p.mission.nodes.base.x, p.mission.nodes.base.y];
    dock_xy = [p.mission.nodes.dock.x, p.mission.nodes.dock.y];
    
    d_base = norm(pos_xy - base_xy);
    d_dock = norm(pos_xy - dock_xy);
    
    %% 3. REQUIRED ENERGIES (With Safety Reserve Margin)
    E_req_dest = e_rate * d_dest * (1.0 + margin);
    E_req_base = e_rate * d_base * (1.0 + margin);
    E_req_dock = e_rate * d_dock * (1.0 + margin);
    
    R_est = E_usable / e_rate; % Estimated achievable range [m]
    
    %% 4. REACHABILITY DETERMINATIONS
    can_reach_dest = (E_usable >= E_req_dest);
    can_reach_dock = (E_usable >= E_req_dock);
    can_reach_base = (E_usable >= E_req_base);
    
    %% 5. MISSION DECISION MODE LOGIC
    % 1=CRUISE (Outbound full speed)
    % 2=ECONOMY (Energy-saving speed)
    % 3=DIVERT_TO_DOCK (Head to charging station)
    % 4=RETURN_TO_BASE (Abort mission and head home)
    % 5=EMERGENCY (Low-power crawl / emergency fallback)
    
    if SOC <= 8.0
        decision_mode = 5; % EMERGENCY
    elseif can_reach_dest
        if SOC >= p.battery.soc_economy_pct && (E_usable >= E_req_dest * 1.5)
            decision_mode = 1; % CRUISE
        else
            decision_mode = 2; % ECONOMY
        end
    elseif can_reach_dock
        decision_mode = 3; % DIVERT_TO_DOCK
    elseif can_reach_base
        decision_mode = 4; % RETURN_TO_BASE
    else
        decision_mode = 5; % EMERGENCY
    end
    
    %% PACK FEASIBILITY REPORT
    feas_report.d_dest_m       = d_dest;
    feas_report.d_base_m       = d_base;
    feas_report.d_dock_m       = d_dock;
    feas_report.E_avail_Wh     = E_avail / 3600.0;
    feas_report.E_usable_Wh    = E_usable / 3600.0;
    feas_report.E_req_dest_Wh  = E_req_dest / 3600.0;
    feas_report.E_req_base_Wh  = E_req_base / 3600.0;
    feas_report.E_req_dock_Wh  = E_req_dock / 3600.0;
    feas_report.R_est_m        = R_est;
    feas_report.can_reach_dest = can_reach_dest;
    feas_report.can_reach_dock = can_reach_dock;
    feas_report.can_reach_base = can_reach_base;
    feas_report.decision_mode  = decision_mode;
end

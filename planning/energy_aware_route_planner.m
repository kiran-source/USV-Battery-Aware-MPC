function [selected_route, candidate_routes] = energy_aware_route_planner(start_pt, goal_pt, map_data, p, plan_mode)
    % ENERGY_AWARE_ROUTE_PLANNER Generates & Compares Shortest vs Energy-Aware vs Dock Routes
    %
    % Inputs:
    %   start_pt   - [x, y] start position [m]
    %   goal_pt    - [x, y] target position [m]
    %   map_data   - Map struct with obstacles and nodes
    %   p          - Full parameter struct (usv, env, bat, mission, mpc)
    %   plan_mode  - 'shortest', 'energy', or 'balanced'
    
    if nargin < 5, plan_mode = 'balanced'; end
    
    u_nominal = p.mission.speeds.u_nominal;
    u_economy = p.mission.speeds.u_economy;
    
    %% 1. CANDIDATE ROUTE 1: Direct A* Shortest-Distance Path
    [path1, ~, ~] = ThetaStarPlanner(start_pt, goal_pt, map_data);
    w_dist_only = struct('w_distance', 1.0, 'w_energy', 0.0, 'w_time', 0.0, 'w_risk', 0.0);
    [cost1_dist, E1_Wh, t1_s, d1_m] = route_cost(path1, u_nominal, p.environment, p.usv, p.battery, w_dist_only);
    [cost1_bal, ~, ~, ~] = route_cost(path1, u_nominal, p.environment, p.usv, p.battery, p.mission.route.weights);
    
    candidate_routes(1).name       = 'Route A (Shortest Distance)';
    candidate_routes(1).path       = path1;
    candidate_routes(1).distance   = d1_m;
    candidate_routes(1).energy_Wh  = E1_Wh;
    candidate_routes(1).travel_time= t1_s;
    candidate_routes(1).cost       = cost1_bal;
    candidate_routes(1).cruise_spd = u_nominal;
    candidate_routes(1).type       = 'DIRECT_DESTINATION';
    
    %% 2. CANDIDATE ROUTE 2: Energy-Aware Sheltered / Favorable Current Path (Via Waypoints)
    % Routes through WP1 and WP2 to gain shelter / favorable flow around shallow reef
    wp1 = [p.mission.nodes.wp1.x, p.mission.nodes.wp1.y];
    wp2 = [p.mission.nodes.wp2.x, p.mission.nodes.wp2.y];
    
    [p2_seg1, ~, ~] = ThetaStarPlanner(start_pt, wp1, map_data);
    [p2_seg2, ~, ~] = ThetaStarPlanner(wp1, wp2, map_data);
    [p2_seg3, ~, ~] = ThetaStarPlanner(wp2, goal_pt, map_data);
    
    path2 = [p2_seg1; p2_seg2(2:end, :); p2_seg3(2:end, :)];
    [~, E2_Wh, t2_s, d2_m] = route_cost(path2, u_economy, p.environment, p.usv, p.battery, p.mission.route.weights);
    [cost2_bal, ~, ~, ~]   = route_cost(path2, u_economy, p.environment, p.usv, p.battery, p.mission.route.weights);
    
    candidate_routes(2).name       = 'Route B (Energy-Optimized Economy)';
    candidate_routes(2).path       = path2;
    candidate_routes(2).distance   = d2_m;
    candidate_routes(2).energy_Wh  = E2_Wh;
    candidate_routes(2).travel_time= t2_s;
    candidate_routes(2).cost       = cost2_bal;
    candidate_routes(2).cruise_spd = u_economy;
    candidate_routes(2).type       = 'SHELTERED_WAYPOINTS';
    
    %% 3. CANDIDATE ROUTE 3: Route via Intermediate Charging Dock
    dock_pt = [p.mission.nodes.dock.x, p.mission.nodes.dock.y];
    [p3_seg1, ~, ~] = ThetaStarPlanner(start_pt, dock_pt, map_data);
    [p3_seg2, ~, ~] = ThetaStarPlanner(dock_pt, goal_pt, map_data);
    path3 = [p3_seg1; p3_seg2(2:end, :)];
    
    [~, E3_Wh, t3_s, d3_m] = route_cost(path3, u_nominal, p.environment, p.usv, p.battery, p.mission.route.weights);
    [cost3_bal, ~, ~, ~]   = route_cost(path3, u_nominal, p.environment, p.usv, p.battery, p.mission.route.weights);
    
    candidate_routes(3).name       = 'Route C (Via Charging Dock)';
    candidate_routes(3).path       = path3;
    candidate_routes(3).distance   = d3_m;
    candidate_routes(3).energy_Wh  = E3_Wh;
    candidate_routes(3).travel_time= t3_s + 600.0; % Includes 10 min dock charging stop
    candidate_routes(3).cost       = cost3_bal + 0.2; % Small penalty for dock overhead
    candidate_routes(3).cruise_spd = u_nominal;
    candidate_routes(3).type       = 'CHARGING_DOCK';
    
    %% 4. ROUTE SELECTION LOGIC
    switch lower(plan_mode)
        case 'shortest'
            [~, best_idx] = min([candidate_routes.distance]);
        case 'energy'
            [~, best_idx] = min([candidate_routes.energy_Wh]);
        otherwise % 'balanced'
            [~, best_idx] = min([candidate_routes.cost]);
    end
    
    selected_route = candidate_routes(best_idx);
end

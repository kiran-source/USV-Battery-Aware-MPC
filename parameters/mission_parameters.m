function p_mis = mission_parameters()
    % MISSION_PARAMETERS Configuration for Map, Waypoints, Docks, and Route Planner
    
    % --- Map Dimensions ---
    p_mis.map.x_min = -200.0; % [m]
    p_mis.map.x_max = 1600.0; % [m]
    p_mis.map.y_min = -300.0; % [m]
    p_mis.map.y_max = 1100.0; % [m]
    p_mis.map.resolution = 10.0; % Grid cell resolution [m]
    
    % --- Key Mission Locations ---
    p_mis.nodes.base.id   = 1;
    p_mis.nodes.base.x    = 0.0;
    p_mis.nodes.base.y    = 0.0;
    p_mis.nodes.base.type = 'BASE';
    
    p_mis.nodes.wp1.id    = 2;
    p_mis.nodes.wp1.x     = 350.0;
    p_mis.nodes.wp1.y     = 250.0;
    p_mis.nodes.wp1.type  = 'WAYPOINT';
    
    p_mis.nodes.wp2.id    = 3;
    p_mis.nodes.wp2.x     = 750.0;
    p_mis.nodes.wp2.y     = 450.0;
    p_mis.nodes.wp2.type  = 'WAYPOINT';
    
    p_mis.nodes.dock.id   = 4;
    p_mis.nodes.dock.x    = 600.0;
    p_mis.nodes.dock.y    = 150.0;
    p_mis.nodes.dock.type = 'CHARGING_DOCK';
    
    p_mis.nodes.harbor.id = 5;
    p_mis.nodes.harbor.x  = 200.0;
    p_mis.nodes.harbor.y  = 600.0;
    p_mis.nodes.harbor.type = 'SAFE_HARBOR';
    
    p_mis.nodes.dest.id   = 6;
    p_mis.nodes.dest.x    = 1200.0;
    p_mis.nodes.dest.y    = 650.0;
    p_mis.nodes.dest.type = 'DESTINATION';
    
    % --- Obstacles / No-Go Islands (Polygons / Circles) ---
    p_mis.obstacles(1).type   = 'circle';
    p_mis.obstacles(1).center = [500.0, 380.0];
    p_mis.obstacles(1).radius = 90.0; % Rocky shoal / shallow reef
    
    p_mis.obstacles(2).type   = 'polygon';
    p_mis.obstacles(2).vertices = [
        850.0, 200.0;
        1050.0, 250.0;
        1000.0, 450.0;
        800.0, 380.0
    ]; % Island / restricted marine sanctuary
    
    p_mis.obstacles(3).type   = 'circle';
    p_mis.obstacles(3).center = [250.0, -100.0];
    p_mis.obstacles(3).radius = 70.0;
    
    % --- Route Planner Multi-Objective Cost Weights ---
    % J_route = w_d * (d / d_ref) + w_e * (E / E_ref) + w_t * (t / t_ref) + w_r * Risk
    p_mis.route.weights.w_distance = 0.35;
    p_mis.route.weights.w_energy   = 0.45;
    p_mis.route.weights.w_time     = 0.15;
    p_mis.route.weights.w_risk     = 0.05;
    
    % --- Mission Guidance & Capture Radii ---
    p_mis.guidance.R_capture_wp   = 15.0;  % Waypoint arrival radius [m]
    p_mis.guidance.R_capture_dest = 8.0;   % Final destination arrival radius [m]
    p_mis.guidance.Delta_LOS      = 25.0;  % Lookahead distance for LOS guidance [m]
    
    % --- Cruise Speeds ---
    p_mis.speeds.u_nominal   = 2.0; % Standard cruise speed [m/s]
    p_mis.speeds.u_economy   = 1.4; % Energy saving cruise speed [m/s]
    p_mis.speeds.u_low_power = 0.9; % Low power crawl speed [m/s]
    p_mis.speeds.u_docking   = 0.4; % Docking approach speed [m/s]
    
    % --- Mission Safety Margins ---
    p_mis.safety_margin = 0.25; % 25% energy reserve margin above baseline
end

function sim_data = run_simulation(p, scenario_cfg)
    % RUN_SIMULATION High-Fidelity 3-DOF Closed-Loop USV Simulation Engine
    %
    % Inputs:
    %   p            - Master parameter struct
    %   scenario_cfg - Scenario configuration:
    %                  .controller_type ('PID', 'CONV_MPC', 'BATTERY_AWARE_MPC')
    %                  .initial_soc     (e.g. 80.0, 60.0, 40.0, 20.0)
    %                  .plan_mode       ('shortest', 'energy', 'balanced')
    %                  .t_max           (Max sim time, e.g. 1200 s)
    %                  .enable_docking  (boolean, true to allow charging stop)
    
    if nargin < 2
        scenario_cfg.controller_type = 'BATTERY_AWARE_MPC';
        scenario_cfg.initial_soc     = 80.0;
        scenario_cfg.plan_mode       = 'balanced';
        scenario_cfg.t_max           = 1200.0;
        scenario_cfg.enable_docking  = false;
    end
    
    dt = 0.1; % Inner integration timestep [s] (10 Hz)
    mpc_decimation = round(p.mpc.Ts / dt); % Outer control update decimation
    
    %% 1. MAP & ROUTE PLANNING
    map_data = build_map(p.mission);
    start_pt = [p.mission.nodes.base.x, p.mission.nodes.base.y];
    goal_pt  = [p.mission.nodes.dest.x, p.mission.nodes.dest.y];
    
    [selected_route, candidate_routes] = energy_aware_route_planner(...
        start_pt, goal_pt, map_data, p, scenario_cfg.plan_mode);
    
    waypoints = selected_route.path;
    N_wp = size(waypoints, 1);
    
    %% 2. INITIALIZE USV STATES
    % Initial position at Base, aligned with first waypoint
    psi0 = atan2(waypoints(2,2) - waypoints(1,2), waypoints(2,1) - waypoints(1,1));
    eta = [waypoints(1,1); waypoints(1,2); psi0]; % [x; y; psi]
    nu  = [0.0; 0.0; 0.0];                          % [u; v; r]
    
    SOC = scenario_cfg.initial_soc;
    E_used = 0.0;
    
    T_cmd = 0.0; N_cmd = 0.0;
    T_L_act = 0.0; T_R_act = 0.0;
    u_ref = selected_route.cruise_spd;
    psi_ref = psi0;
    e_y = 0.0; e_psi = 0.0;
    
    wp_idx = 2;
    mission_state = 1; % 1=GO_TO_WP, 2=GO_TO_DEST, 3=CHARGING, 4=RETURN_TO_BASE, 5=EMERGENCY, 6=COMPLETE
    is_docked = false;
    dock_time_elapsed = 0.0;
    
    state_pid = [];
    
    % Preallocate log arrays
    N_steps = ceil(scenario_cfg.t_max / dt);
    t_log       = zeros(N_steps, 1);
    x_log       = zeros(N_steps, 1);
    y_log       = zeros(N_steps, 1);
    psi_log     = zeros(N_steps, 1);
    u_log       = zeros(N_steps, 1);
    v_log       = zeros(N_steps, 1);
    r_log       = zeros(N_steps, 1);
    u_ref_log   = zeros(N_steps, 1);
    psi_ref_log = zeros(N_steps, 1);
    e_y_log     = zeros(N_steps, 1);
    e_psi_log   = zeros(N_steps, 1);
    T_log       = zeros(N_steps, 1);
    N_mom_log   = zeros(N_steps, 1);
    T_L_log     = zeros(N_steps, 1);
    T_R_log     = zeros(N_steps, 1);
    SOC_log     = zeros(N_steps, 1);
    P_log       = zeros(N_steps, 1);
    E_used_log  = zeros(N_steps, 1);
    V_term_log  = zeros(N_steps, 1);
    mode_log    = zeros(N_steps, 1);
    state_log   = zeros(N_steps, 1);
    feas_log    = zeros(N_steps, 1);
    R_est_log   = zeros(N_steps, 1);
    d_dest_log  = zeros(N_steps, 1);
    
    k = 0;
    t = 0.0;
    mission_complete = false;
    
    %% 3. MAIN SIMULATION LOOP (RK4 / Euler)
    while ~mission_complete && k < N_steps
        k = k + 1;
        t = (k-1) * dt;
        
        pos_xy = eta(1:2)';
        
        %% A. GUIDANCE & SUPERVISORY DECISION (Every control step)
        if mod(k-1, mpc_decimation) == 0
            % Waypoint Manager
            [wp_idx, ~, ~, dist_to_wp, is_dest_reached] = waypoint_manager(...
                pos_xy, waypoints, wp_idx, p.mission);
            
            % Feasibility & Reachability Evaluator
            P_curr = (T_cmd * max(nu(1), 0.1)) / p.usv.prop.eta_motor + p.battery.P_hotel;
            [feas_report, decision_mode] = mission_feasibility(...
                pos_xy, SOC, P_curr, nu(1), waypoints, wp_idx, p);
            
            % Check Charging Dock Trigger
            if scenario_cfg.enable_docking && (decision_mode == 3) && ~is_docked
                dock_xy = [p.mission.nodes.dock.x, p.mission.nodes.dock.y];
                if norm(pos_xy - dock_xy) <= 15.0
                    is_docked = true;
                    dock_time_elapsed = 0.0;
                end
            end
            
            % Check Return to Base Trigger
            if decision_mode == 4 && mission_state ~= 4
                % Re-plan path directly to Base
                base_xy = [p.mission.nodes.base.x, p.mission.nodes.base.y];
                [return_path, ~, ~] = ThetaStarPlanner(pos_xy, base_xy, map_data);
                waypoints = return_path;
                wp_idx = 2;
                mission_state = 4; % RETURN_TO_BASE
            end
            
            % Docking State Execution
            if is_docked
                dock_time_elapsed = dock_time_elapsed + p.mpc.Ts;
                if dock_time_elapsed >= 300.0 % 5 minutes charge
                    is_docked = false;
                    % Resume path to destination
                    [resume_path, ~, ~] = ThetaStarPlanner(pos_xy, goal_pt, map_data);
                    waypoints = resume_path;
                    wp_idx = 2;
                end
            end
            
            % Path Following & Setpoints
            if is_docked
                u_ref = 0.0;
                psi_ref = eta(3);
                e_y = 0.0; e_psi = 0.0;
            else
                [u_ref, psi_ref, e_y, e_psi] = path_following(...
                    eta, nu, waypoints, wp_idx, decision_mode, p);
            end
            
            % Destination Arrival Check
            if is_dest_reached && ~is_docked
                mission_complete = true;
                mission_state = 6; % COMPLETE
            end
            
            % Battery Depletion Emergency Check
            if SOC <= 0.05
                mission_complete = true;
                mission_state = 5; % STRANDED / EMERGENCY
            end
            
            %% B. CONTROLLER EXECUTION
            if is_docked || mission_complete
                T_cmd = 0.0; N_cmd = 0.0;
            else
                switch upper(scenario_cfg.controller_type)
                    case 'PID'
                        [T_cmd, N_cmd, state_pid] = PID_controller(...
                            u_ref, psi_ref, nu(1), eta(3), nu(3), p.mpc.Ts, state_pid, p);
                        
                    case 'PID_BATTERY'
                        [T_cmd, N_cmd, state_pid] = PID_battery_aware(...
                            u_ref, psi_ref, nu(1), eta(3), nu(3), SOC, p.mpc.Ts, state_pid, p);
                        
                    case 'PID_ML'
                        [T_cmd, N_cmd, state_pid] = PID_ML_controller(...
                            u_ref, psi_ref, nu(1), eta(3), nu(3), p.mpc.Ts, state_pid, p);
                        
                    case 'PID_ML_BATTERY'
                        [T_cmd, N_cmd, state_pid] = PID_ML_battery_aware(...
                            u_ref, psi_ref, nu(1), eta(3), nu(3), SOC, p.mpc.Ts, state_pid, p);
                        
                    case 'CONV_MPC'
                        [T_cmd, N_cmd, ~] = conventional_MPC(...
                            eta, nu, u_ref, psi_ref, e_y, T_cmd, N_cmd, p);
                        
                    case 'MPC_ML'
                        if isfield(p, 'ml_ann') && ~isempty(p.ml_ann)
                            input_ann = [nu(1), u_ref, wrapToPi(psi_ref - eta(3)), nu(3), SOC, e_y];
                            [~, mpc_w] = predict_neural_network(input_ann, p.ml_ann);
                            p.mpc.weights.w_speed = mpc_w(1);
                            p.mpc.weights.w_cte = mpc_w(2);
                            p.mpc.weights.w_psi = mpc_w(3);
                            p.mpc.weights.w_yaw_rate = mpc_w(4);
                            p.mpc.weights.w_power = mpc_w(5);
                            p.mpc.weights.w_rate_T = mpc_w(6);
                            p.mpc.weights.w_rate_N = mpc_w(7);
                        end
                        [T_cmd, N_cmd, ~] = conventional_MPC(...
                            eta, nu, u_ref, psi_ref, e_y, T_cmd, N_cmd, p);
                        
                    case 'MPC_ML_BATTERY'
                        if isfield(p, 'ml_ann') && ~isempty(p.ml_ann)
                            input_ann = [nu(1), u_ref, wrapToPi(psi_ref - eta(3)), nu(3), SOC, e_y];
                            [~, mpc_w] = predict_neural_network(input_ann, p.ml_ann);
                            p.mpc.weights.w_speed = mpc_w(1);
                            p.mpc.weights.w_cte = mpc_w(2);
                            p.mpc.weights.w_psi = mpc_w(3);
                            p.mpc.weights.w_yaw_rate = mpc_w(4);
                            p.mpc.weights.w_power = mpc_w(5);
                            p.mpc.weights.w_rate_T = mpc_w(6);
                            p.mpc.weights.w_rate_N = mpc_w(7);
                        end
                        [T_cmd, N_cmd, ~] = battery_aware_MPC(...
                            eta, nu, u_ref, psi_ref, e_y, SOC, T_cmd, N_cmd, p);
                        
                    otherwise % 'BATTERY_AWARE_MPC'
                        [T_cmd, N_cmd, ~] = battery_aware_MPC(...
                            eta, nu, u_ref, psi_ref, e_y, SOC, T_cmd, N_cmd, p);
                end
            end
        end
        
        %% C. PROPULSION ACTUATORS
        [tau_ctrl, P_prop, T_L_act, T_R_act] = propulsion_model(...
            T_cmd, N_cmd, nu(1), T_L_act, T_R_act, dt, p.usv);
        
        %% D. BATTERY DYNAMICS
        [SOC, E_used, P_batt, V_term, ~] = battery_model(...
            P_prop, is_docked, SOC, E_used, dt, p.battery);
        
        %% E. 3-DOF USV INTEGRATION (RK4)
        [k1_eta, k1_nu] = USV_3DOF_dynamics(eta, nu, tau_ctrl, t, p.usv, p.environment);
        
        eta_k2 = eta + 0.5 * dt * k1_eta;
        nu_k2  = nu  + 0.5 * dt * k1_nu;
        [k2_eta, k2_nu] = USV_3DOF_dynamics(eta_k2, nu_k2, tau_ctrl, t + 0.5*dt, p.usv, p.environment);
        
        eta_k3 = eta + 0.5 * dt * k2_eta;
        nu_k3  = nu  + 0.5 * dt * k2_nu;
        [k3_eta, k3_nu] = USV_3DOF_dynamics(eta_k3, nu_k3, tau_ctrl, t + 0.5*dt, p.usv, p.environment);
        
        eta_k4 = eta + dt * k3_eta;
        nu_k4  = nu  + dt * k3_nu;
        [k4_eta, k4_nu] = USV_3DOF_dynamics(eta_k4, nu_k4, tau_ctrl, t + dt, p.usv, p.environment);
        
        eta = eta + (dt / 6.0) * (k1_eta + 2*k2_eta + 2*k3_eta + k4_eta);
        nu  = nu  + (dt / 6.0) * (k1_nu  + 2*k2_nu  + 2*k3_nu  + k4_nu);
        
        % Normalize heading angle
        eta(3) = wrapToPi(eta(3));
        
        %% F. LOGGING
        t_log(k)       = t;
        x_log(k)       = eta(1);
        y_log(k)       = eta(2);
        psi_log(k)     = eta(3);
        u_log(k)       = nu(1);
        v_log(k)       = nu(2);
        r_log(k)       = nu(3);
        u_ref_log(k)   = u_ref;
        psi_ref_log(k) = psi_ref;
        e_y_log(k)     = e_y;
        e_psi_log(k)   = e_psi;
        T_log(k)       = tau_ctrl(1);
        N_mom_log(k)   = tau_ctrl(3);
        T_L_log(k)     = T_L_act;
        T_R_log(k)     = T_R_act;
        SOC_log(k)     = SOC;
        P_log(k)       = P_batt;
        E_used_log(k)  = E_used;
        V_term_log(k)  = V_term;
        mode_log(k)    = decision_mode;
        state_log(k)   = mission_state;
        feas_log(k)    = double(feas_report.can_reach_dest);
        R_est_log(k)   = feas_report.R_est_m;
        d_dest_log(k)  = feas_report.d_dest_m;
    end
    
    %% 4. PACKAGE OUTPUT TELEMETRY
    sim_data.t           = t_log(1:k);
    sim_data.x           = x_log(1:k);
    sim_data.y           = y_log(1:k);
    sim_data.psi         = psi_log(1:k);
    sim_data.u           = u_log(1:k);
    sim_data.v           = v_log(1:k);
    sim_data.r           = r_log(1:k);
    sim_data.u_ref       = u_ref_log(1:k);
    sim_data.psi_ref     = psi_ref_log(1:k);
    sim_data.e_y         = e_y_log(1:k);
    sim_data.e_psi       = e_psi_log(1:k);
    sim_data.T           = T_log(1:k);
    sim_data.N           = N_mom_log(1:k);
    sim_data.T_L         = T_L_log(1:k);
    sim_data.T_R         = T_R_log(1:k);
    sim_data.SOC         = SOC_log(1:k);
    sim_data.P           = P_log(1:k);
    sim_data.E_used_J    = E_used_log(1:k);
    sim_data.E_used_Wh   = E_used_log(1:k) / 3600.0;
    sim_data.V_term      = V_term_log(1:k);
    sim_data.mode        = mode_log(1:k);
    sim_data.state       = state_log(1:k);
    sim_data.is_feasible = feas_log(1:k);
    sim_data.R_est       = R_est_log(1:k);
    sim_data.d_dest      = d_dest_log(1:k);
    sim_data.waypoints   = waypoints;
    sim_data.map_data    = map_data;
    sim_data.route       = selected_route;
    sim_data.candidate_routes = candidate_routes;
    
    % Summary metrics
    sim_data.summary.travel_time_s = t_log(k);
    sim_data.summary.total_dist_m  = sum(sqrt(diff(x_log(1:k)).^2 + diff(y_log(1:k)).^2));
    sim_data.summary.final_soc     = SOC_log(k);
    sim_data.summary.energy_Wh     = E_used_log(k) / 3600.0;
    sim_data.summary.avg_power_W   = mean(P_log(1:k));
    sim_data.summary.peak_power_W  = max(P_log(1:k));
    sim_data.summary.rmse_u        = sqrt(mean((u_log(1:k) - u_ref_log(1:k)).^2));
    sim_data.summary.rmse_cte      = sqrt(mean(e_y_log(1:k).^2));
    sim_data.summary.rmse_psi      = sqrt(mean(e_psi_log(1:k).^2));
    sim_data.summary.is_complete   = (norm([x_log(k) - goal_pt(1), y_log(k) - goal_pt(2)]) <= p.mission.guidance.R_capture_dest);
    sim_data.summary.is_feasible   = feas_log(max(k-10, 1));
end

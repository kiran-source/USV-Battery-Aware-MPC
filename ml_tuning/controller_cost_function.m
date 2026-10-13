function cost = controller_cost_function(params, controller_type, p_base)
    % CONTROLLER_COST_FUNCTION Evaluates controller performance for PSO tuning
    %
    % Runs a shortened simulation with given parameters and returns a
    % multi-objective cost (lower = better).
    %
    % Cost = w1*RMSE_speed + w2*RMSE_cte + w3*Energy_normalized + w4*Penalty
    
    p = p_base;
    
    %% 1. MAP PARAMETERS FROM PSO VECTOR TO CONTROLLER STRUCT
    switch upper(controller_type)
        case 'PID'
            % params = [Kp_u, Ki_u, Kd_u, Kp_psi, Ki_psi, Kd_psi]
            p.ml_pid.Kp_u   = params(1);
            p.ml_pid.Ki_u   = params(2);
            p.ml_pid.Kd_u   = params(3);
            p.ml_pid.Kp_psi = params(4);
            p.ml_pid.Ki_psi = params(5);
            p.ml_pid.Kd_psi = params(6);
            cfg.controller_type = 'PID_ML';
            
        case 'PID_BATTERY'
            % params = [Kp_u, Ki_u, Kd_u, Kp_psi, Ki_psi, Kd_psi]
            p.ml_pid.Kp_u   = params(1);
            p.ml_pid.Ki_u   = params(2);
            p.ml_pid.Kd_u   = params(3);
            p.ml_pid.Kp_psi = params(4);
            p.ml_pid.Ki_psi = params(5);
            p.ml_pid.Kd_psi = params(6);
            cfg.controller_type = 'PID_ML_BATTERY';
            
        case 'MPC'
            % params = [w_speed, w_cte, w_psi, w_yaw_rate, w_rate_T, w_rate_N]
            p.mpc.weights.w_speed    = params(1);
            p.mpc.weights.w_cte      = params(2);
            p.mpc.weights.w_psi      = params(3);
            p.mpc.weights.w_yaw_rate = params(4);
            p.mpc.weights.w_rate_T   = params(5);
            p.mpc.weights.w_rate_N   = params(6);
            cfg.controller_type = 'MPC_ML';
            
        case 'MPC_BATTERY'
            % params = [w_speed, w_cte, w_psi, w_yaw_rate, w_power, w_rate_T, w_rate_N]
            p.mpc.weights.w_speed    = params(1);
            p.mpc.weights.w_cte      = params(2);
            p.mpc.weights.w_psi      = params(3);
            p.mpc.weights.w_yaw_rate = params(4);
            p.mpc.weights.w_power    = params(5);
            p.mpc.weights.w_rate_T   = params(6);
            p.mpc.weights.w_rate_N   = params(7);
            cfg.controller_type = 'MPC_ML_BATTERY';
    end
    
    %% 2. RUN SHORTENED SIMULATION (faster evaluation for PSO)
    cfg.initial_soc    = 80.0;
    cfg.plan_mode      = 'balanced';
    cfg.t_max = 1100.0;  % Shortened for fast evaluation
    cfg.enable_docking = false;
    
    % Disable environmental disturbances for clean tuning
    p.environment.current.enabled = false;
    p.environment.wind.enabled    = false;
    p.environment.wave.enabled    = false;
    
    try
        sim_data = run_simulation(p, cfg);
    catch
        cost = 1e6; % Penalize crashes
        return;
    end
    
    %% 3. COMPUTE MULTI-OBJECTIVE COST
    rmse_speed = sim_data.summary.rmse_u;
    rmse_cte   = sim_data.summary.rmse_cte;
    energy_Wh  = sim_data.summary.energy_Wh;
    final_soc  = sim_data.summary.final_soc;
    travel_time = sim_data.summary.travel_time_s;
    is_complete = sim_data.summary.is_complete;
    
    % Normalize metrics
    energy_norm = energy_Wh / 200.0;   % Normalize to ~200 Wh baseline
    time_norm   = travel_time / 500.0;  % Normalize to max time
    
    % ============================================================
% MISSION-SUCCESS-FIRST COST FUNCTION
% ============================================================

% Base performance cost
cost = 3.0 * rmse_speed ...
     + 5.0 * rmse_cte ...
     + 1.5 * energy_norm ...
     + 0.5 * time_norm;

% Strong penalty for mission failure
if ~is_complete
    cost = cost + 100.0;
end

% Penalize very large tracking error
if rmse_cte > 10.0
    cost = cost + 10.0 * (rmse_cte - 10.0);
end

% Penalize low final SOC
if final_soc < 50.0
    cost = cost + 2.0 * (50.0 - final_soc) / 50.0;
end

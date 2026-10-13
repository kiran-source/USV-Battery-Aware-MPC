function all_results = run_all_experiments()
    % RUN_ALL_EXPERIMENTS Executes All 15 Required Benchmark Experiments
    
    clc;
    fprintf('========================================================================\n');
    fprintf('   USV 3-DOF BATTERY-AWARE MPC: 15-SCENARIO BENCHMARK EXECUTION SUITE   \n');
    fprintf('========================================================================\n\n');
    
    p_base = load_all_parameters();
    all_results = cell(15, 1);
    
    %% --- EXP 1: PID, 80% SOC, No Disturbance ---
    fprintf('[1/15] Running Exp 1: PID (80%% SOC, Calibrated Baseline)...\n');
    p = p_base;
    p.environment.current.enabled = false;
    p.environment.wind.enabled    = false;
    p.environment.wave.enabled    = false;
    cfg.controller_type = 'PID';
    cfg.initial_soc     = 80.0;
    cfg.plan_mode       = 'balanced';
    cfg.t_max           = 1000.0;
    cfg.enable_docking  = false;
    res1 = run_simulation(p, cfg);
    res1.name = 'Exp 1: PID Baseline (80% SOC, Calm)';
    all_results{1} = res1;
    
    %% --- EXP 2: Conventional MPC, 80% SOC, No Disturbance ---
    fprintf('[2/15] Running Exp 2: Conventional MPC (80%% SOC, Calm)...\n');
    p = p_base;
    p.environment.current.enabled = false;
    p.environment.wind.enabled    = false;
    p.environment.wave.enabled    = false;
    cfg.controller_type = 'CONV_MPC';
    cfg.initial_soc     = 80.0;
    res2 = run_simulation(p, cfg);
    res2.name = 'Exp 2: Conventional MPC (80% SOC, Calm)';
    all_results{2} = res2;
    
    %% --- EXP 3: Battery-Aware MPC, 80% SOC, No Disturbance ---
    fprintf('[3/15] Running Exp 3: Battery-Aware MPC (80%% SOC, Calm)...\n');
    p = p_base;
    p.environment.current.enabled = false;
    p.environment.wind.enabled    = false;
    p.environment.wave.enabled    = false;
    cfg.controller_type = 'BATTERY_AWARE_MPC';
    cfg.initial_soc     = 80.0;
    res3 = run_simulation(p, cfg);
    res3.name = 'Exp 3: Battery-Aware MPC (80% SOC, Calm)';
    all_results{3} = res3;
    
    %% --- EXP 4: Battery-Aware MPC, 60% SOC ---
    fprintf('[4/15] Running Exp 4: Battery-Aware MPC (60%% SOC)...\n');
    p = p_base;
    cfg.controller_type = 'BATTERY_AWARE_MPC';
    cfg.initial_soc     = 60.0;
    res4 = run_simulation(p, cfg);
    res4.name = 'Exp 4: Battery-Aware MPC (60% SOC)';
    all_results{4} = res4;
    
    %% --- EXP 5: Battery-Aware MPC, 40% SOC ---
    fprintf('[5/15] Running Exp 5: Battery-Aware MPC (40%% SOC)...\n');
    p = p_base;
    cfg.initial_soc     = 40.0;
    res5 = run_simulation(p, cfg);
    res5.name = 'Exp 5: Battery-Aware MPC (40% SOC)';
    all_results{5} = res5;
    
    %% --- EXP 6: Battery-Aware MPC, 20% SOC ---
    fprintf('[6/15] Running Exp 6: Battery-Aware MPC (20%% SOC - Preservation Mode)...\n');
    p = p_base;
    cfg.initial_soc     = 20.0;
    res6 = run_simulation(p, cfg);
    res6.name = 'Exp 6: Battery-Aware MPC (20% SOC)';
    all_results{6} = res6;
    
    %% --- EXP 7: Battery-Aware MPC with Opposing Current ---
    fprintf('[7/15] Running Exp 7: Battery-Aware MPC (Opposing Current 0.4 m/s)...\n');
    p = p_base;
    p.environment.current.enabled   = true;
    p.environment.current.speed     = 0.45;
    p.environment.current.direction = deg2rad(180); % Opposing
    p.environment.wind.enabled      = false;
    p.environment.wave.enabled      = false;
    cfg.initial_soc     = 80.0;
    res7 = run_simulation(p, cfg);
    res7.name = 'Exp 7: Battery-Aware MPC (Opposing Current)';
    all_results{7} = res7;
    
    %% --- EXP 8: Battery-Aware MPC with Cross-Current ---
    fprintf('[8/15] Running Exp 8: Battery-Aware MPC (Cross-Current 0.4 m/s)...\n');
    p = p_base;
    p.environment.current.enabled   = true;
    p.environment.current.speed     = 0.40;
    p.environment.current.direction = deg2rad(90); % Cross-current (North)
    p.environment.wind.enabled      = false;
    p.environment.wave.enabled      = false;
    cfg.initial_soc     = 80.0;
    res8 = run_simulation(p, cfg);
    res8.name = 'Exp 8: Battery-Aware MPC (Cross-Current)';
    all_results{8} = res8;
    
    %% --- EXP 9: Battery-Aware MPC with Wind ---
    fprintf('[9/15] Running Exp 9: Battery-Aware MPC (Wind 6.0 m/s SW)...\n');
    p = p_base;
    p.environment.current.enabled = false;
    p.environment.wind.enabled    = true;
    p.environment.wind.speed      = 6.0;
    p.environment.wave.enabled    = false;
    cfg.initial_soc     = 80.0;
    res9 = run_simulation(p, cfg);
    res9.name = 'Exp 9: Battery-Aware MPC (Wind Disturbance)';
    all_results{9} = res9;
    
    %% --- EXP 10: Battery-Aware MPC with Combined Disturbances ---
    fprintf('[10/15] Running Exp 10: Battery-Aware MPC (Combined Current + Wind + Waves)...\n');
    p = p_base;
    p.environment.current.enabled = true;
    p.environment.wind.enabled    = true;
    p.environment.wave.enabled    = true;
    cfg.initial_soc     = 80.0;
    res10 = run_simulation(p, cfg);
    res10.name = 'Exp 10: Battery-Aware MPC (Full Marine Environment)';
    all_results{10} = res10;
    
    %% --- EXP 11: Shortest Route vs Energy-Aware Route ---
    fprintf('[11/15] Running Exp 11: Shortest-Distance Route Mode...\n');
    p = p_base;
    cfg.plan_mode   = 'shortest';
    cfg.initial_soc = 80.0;
    res11 = run_simulation(p, cfg);
    res11.name = 'Exp 11: Shortest-Distance Route Selection';
    all_results{11} = res11;
    
    %% --- EXP 12: Destination Reachable vs Not Reachable ---
    fprintf('[12/15] Running Exp 12: Extreme Depleted Battery (12%% Initial SOC)...\n');
    p = p_base;
    cfg.plan_mode   = 'energy';
    cfg.initial_soc = 12.0;
    res12 = run_simulation(p, cfg);
    res12.name = 'Exp 12: Infeasible Destination Detection (12% SOC)';
    all_results{12} = res12;
    
    %% --- EXP 13: Destination Reachable Only Through Charging Dock ---
    fprintf('[13/15] Running Exp 13: Mission via Charging Dock (30%% Initial SOC)...\n');
    p = p_base;
    cfg.plan_mode       = 'balanced';
    cfg.initial_soc     = 30.0;
    cfg.enable_docking  = true;
    res13 = run_simulation(p, cfg);
    res13.name = 'Exp 13: Charging Dock Refueling Mission';
    all_results{13} = res13;
    
    %% --- EXP 14: Low Battery Requiring Return to Base ---
    fprintf('[14/15] Running Exp 14: Return to Base Abort (15%% Initial SOC)...\n');
    p = p_base;
    cfg.plan_mode       = 'balanced';
    cfg.initial_soc     = 15.0;
    cfg.enable_docking  = false;
    res14 = run_simulation(p, cfg);
    res14.name = 'Exp 14: Autonomous Return-to-Base Abort';
    all_results{14} = res14;
    
    %% --- EXP 15: Obstacle Map Route Planning ---
    fprintf('[15/15] Running Exp 15: Obstacle Navigation Route Tracking...\n');
    p = p_base;
    cfg.plan_mode   = 'balanced';
    cfg.initial_soc = 80.0;
    res15 = run_simulation(p, cfg);
    res15.name = 'Exp 15: Obstacle Avoidance & Path Tracking';
    all_results{15} = res15;
    
    fprintf('\n========================================================================\n');
    fprintf('            ALL 15 EXPERIMENTS COMPLETED SUCCESSFULLY!                  \n');
    fprintf('========================================================================\n\n');
end

function build_USV_model()
    % BUILD_USV_MODEL Programmatically constructs the complete Simulink model:
    % 'USV_Battery_MPC_First_Evaluation.slx'
    %
    % Subsystems created:
    %   1. Navigation
    %   2. USV Dynamics
    %   3. PID Controller
    %   4. MPC Controller
    %   5. Battery Model
    %   6. Energy Calculation
    %   7. Mission Feasibility
    %   8. Data Logging

    model_name = 'USV_Battery_MPC_First_Evaluation';
    
    % Close system if already open without saving changes to previous instance
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    
    % Create a new empty Simulink system
    new_system(model_name);
    open_system(model_name);

    % Configure solver settings for stable, accurate continuous/discrete co-simulation
    set_param(model_name, 'SolverType', 'Variable-step');
    set_param(model_name, 'Solver', 'ode45');
    set_param(model_name, 'StartTime', '0.0');
    set_param(model_name, 'StopTime', '1100.0');
    set_param(model_name, 'MaxStep', '0.1');
    set_param(model_name, 'SaveTime', 'on');
    set_param(model_name, 'TimeSaveName', 'tout');

    % Pre-populate Model Workspace with default parameters
    p_init = parameters();
    mdlWk = get_param(model_name, 'ModelWorkspace');
    mdlWk.assignin('params', p_init);
    mdlWk.assignin('controller_mode', 1);
    mdlWk.assignin('battery_aware_enable', 1);
    mdlWk.assignin('initial_soc', 80.0);

    sf = sfroot;

    %% -------------------------------------------------------------
    %% 1. NAVIGATION SUBSYSTEM
    %% -------------------------------------------------------------
    nav_sys = [model_name, '/Navigation'];
    add_block('simulink/Ports & Subsystems/Subsystem', nav_sys, 'Position', [50, 100, 200, 200]);
    clear_subsystem(nav_sys);

    % Ports
    add_block('simulink/Sources/In1', [nav_sys, '/Position'], 'Position', [30, 40, 60, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [nav_sys, '/SOC'], 'Position', [30, 80, 60, 94], 'Port', '2');
    add_block('simulink/Sinks/Out1', [nav_sys, '/u_ref'], 'Position', [480, 40, 510, 54], 'Port', '1');
    add_block('simulink/Sinks/Out1', [nav_sys, '/Distance_Remaining'], 'Position', [480, 90, 510, 104], 'Port', '2');
    add_block('simulink/Sinks/Out1', [nav_sys, '/Mission_Done'], 'Position', [480, 140, 510, 154], 'Port', '3');

    % MATLAB Function for Navigation Logic
    add_block('simulink/User-Defined Functions/MATLAB Function', [nav_sys, '/Nav_Logic'], 'Position', [150, 30, 380, 160]);
    ch_nav = sf.find('Path', [nav_sys, '/Nav_Logic'], '-isa', 'Stateflow.EMChart');
    ch_nav.Script = sprintf([...
        'function [u_ref, dist_rem, arrived] = Nav_Logic(pos, soc, x_dest, arrival_tol, soc_th_high, soc_th_low, sp_high, sp_med, sp_low, bat_aware_en)\n', ...
        'u_ref = 0.0;\n', ...
        'dist_rem = 0.0;\n', ...
        'arrived = 0.0;\n', ...
        'dist_rem = max(0.0, x_dest - pos);\n', ...
        'if dist_rem <= arrival_tol\n', ...
        '    u_ref = 0.0;\n', ...
        '    arrived = 1.0;\n', ...
        'else\n', ...
        '    arrived = 0.0;\n', ...
        '    if bat_aware_en > 0.5\n', ...
        '        if soc > soc_th_high\n', ...
        '            u_ref = sp_high;\n', ...
        '        elseif soc >= soc_th_low\n', ...
        '            u_ref = sp_med;\n', ...
        '        else\n', ...
        '            u_ref = sp_low;\n', ...
        '        end\n', ...
        '    else\n', ...
        '        u_ref = sp_high;\n', ...
        '    end\n', ...
        'end\n']);

    % Parameters / Constants feeding Nav_Logic
    add_block('simulink/Sources/Constant', [nav_sys, '/x_dest'], 'Position', [30, 120, 80, 134], 'Value', 'params.mission.x_dest');
    add_block('simulink/Sources/Constant', [nav_sys, '/arrival_tol'], 'Position', [30, 150, 80, 164], 'Value', 'params.mission.arrival_tol');
    add_block('simulink/Sources/Constant', [nav_sys, '/soc_th_high'], 'Position', [30, 180, 80, 194], 'Value', 'params.battery_aware.soc_threshold_high');
    add_block('simulink/Sources/Constant', [nav_sys, '/soc_th_low'], 'Position', [30, 210, 80, 224], 'Value', 'params.battery_aware.soc_threshold_low');
    add_block('simulink/Sources/Constant', [nav_sys, '/sp_high'], 'Position', [30, 240, 80, 254], 'Value', 'params.battery_aware.speed_high');
    add_block('simulink/Sources/Constant', [nav_sys, '/sp_med'], 'Position', [30, 270, 80, 284], 'Value', 'params.battery_aware.speed_med');
    add_block('simulink/Sources/Constant', [nav_sys, '/sp_low'], 'Position', [30, 300, 80, 314], 'Value', 'params.battery_aware.speed_low');
    add_block('simulink/Sources/Constant', [nav_sys, '/bat_aware_en'], 'Position', [30, 330, 80, 344], 'Value', 'battery_aware_enable');

    % Connections inside Navigation
    add_line(nav_sys, 'Position/1', 'Nav_Logic/1');
    add_line(nav_sys, 'SOC/1', 'Nav_Logic/2');
    add_line(nav_sys, 'x_dest/1', 'Nav_Logic/3');
    add_line(nav_sys, 'arrival_tol/1', 'Nav_Logic/4');
    add_line(nav_sys, 'soc_th_high/1', 'Nav_Logic/5');
    add_line(nav_sys, 'soc_th_low/1', 'Nav_Logic/6');
    add_line(nav_sys, 'sp_high/1', 'Nav_Logic/7');
    add_line(nav_sys, 'sp_med/1', 'Nav_Logic/8');
    add_line(nav_sys, 'sp_low/1', 'Nav_Logic/9');
    add_line(nav_sys, 'bat_aware_en/1', 'Nav_Logic/10');

    add_line(nav_sys, 'Nav_Logic/1', 'u_ref/1');
    add_line(nav_sys, 'Nav_Logic/2', 'Distance_Remaining/1');
    add_line(nav_sys, 'Nav_Logic/3', 'Mission_Done/1');

    %% -------------------------------------------------------------
    %% 2. PID CONTROLLER SUBSYSTEM
    %% -------------------------------------------------------------
    pid_sys = [model_name, '/PID_Controller'];
    add_block('simulink/Ports & Subsystems/Subsystem', pid_sys, 'Position', [300, 50, 440, 140]);
    clear_subsystem(pid_sys);

    add_block('simulink/Sources/In1', [pid_sys, '/u_ref'], 'Position', [30, 40, 60, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [pid_sys, '/u_meas'], 'Position', [30, 100, 60, 114], 'Port', '2');
    add_block('simulink/Sinks/Out1', [pid_sys, '/Thrust_PID'], 'Position', [460, 70, 490, 84], 'Port', '1');

    % Error: u_ref - u_meas
    add_block('simulink/Math Operations/Sum', [pid_sys, '/Sum_Err'], 'Position', [100, 40, 120, 70], 'Inputs', '+-');
    
    % Proportional path
    add_block('simulink/Math Operations/Gain', [pid_sys, '/Kp_Gain'], 'Position', [160, 25, 210, 55], 'Gain', 'params.pid.Kp');
    
    % Integral path with anti-windup clamping
    add_block('simulink/Math Operations/Gain', [pid_sys, '/Ki_Gain'], 'Position', [160, 75, 210, 105], 'Gain', 'params.pid.Ki');
    add_block('simulink/Continuous/Integrator', [pid_sys, '/I_Int'], 'Position', [240, 75, 270, 105], ...
        'LimitOutput', 'on', ...
        'UpperSaturationLimit', 'params.usv.T_max', ...
        'LowerSaturationLimit', 'params.usv.T_min');

    % Derivative path
    add_block('simulink/Math Operations/Gain', [pid_sys, '/Kd_Gain'], 'Position', [160, 130, 210, 160], 'Gain', 'params.pid.Kd');
    add_block('simulink/Continuous/Derivative', [pid_sys, '/D_Deriv'], 'Position', [240, 130, 270, 160]);

    % Sum PID terms and Saturate
    add_block('simulink/Math Operations/Sum', [pid_sys, '/Sum_PID'], 'Position', [320, 60, 340, 95], 'Inputs', '+++');
    add_block('simulink/Discontinuities/Saturation', [pid_sys, '/Sat_Thrust'], 'Position', [380, 65, 410, 95], ...
        'UpperLimit', 'params.usv.T_max', ...
        'LowerLimit', 'params.usv.T_min');

    add_line(pid_sys, 'u_ref/1', 'Sum_Err/1');
    add_line(pid_sys, 'u_meas/1', 'Sum_Err/2');
    
    add_line(pid_sys, 'Sum_Err/1', 'Kp_Gain/1');
    add_line(pid_sys, 'Sum_Err/1', 'Ki_Gain/1');
    add_line(pid_sys, 'Sum_Err/1', 'Kd_Gain/1');

    add_line(pid_sys, 'Ki_Gain/1', 'I_Int/1');
    add_line(pid_sys, 'Kd_Gain/1', 'D_Deriv/1');

    add_line(pid_sys, 'Kp_Gain/1', 'Sum_PID/1');
    add_line(pid_sys, 'I_Int/1', 'Sum_PID/2');
    add_line(pid_sys, 'D_Deriv/1', 'Sum_PID/3');

    add_line(pid_sys, 'Sum_PID/1', 'Sat_Thrust/1');
    add_line(pid_sys, 'Sat_Thrust/1', 'Thrust_PID/1');

    %% -------------------------------------------------------------
    %% 3. MPC CONTROLLER SUBSYSTEM
    %% -------------------------------------------------------------
    mpc_sys = [model_name, '/MPC_Controller'];
    add_block('simulink/Ports & Subsystems/Subsystem', mpc_sys, 'Position', [300, 180, 440, 270]);
    clear_subsystem(mpc_sys);

    add_block('simulink/Sources/In1', [mpc_sys, '/u_ref'], 'Position', [30, 40, 60, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [mpc_sys, '/u_meas'], 'Position', [30, 90, 60, 104], 'Port', '2');
    add_block('simulink/Sinks/Out1', [mpc_sys, '/Thrust_MPC'], 'Position', [480, 60, 510, 74], 'Port', '1');

    % Zero-Order Hold blocks to enforce discrete sample time
    add_block('simulink/Discrete/Zero-Order Hold', [mpc_sys, '/ZOH_ref'], 'Position', [100, 35, 135, 65], 'SampleTime', 'params.mpc.Ts');
    add_block('simulink/Discrete/Zero-Order Hold', [mpc_sys, '/ZOH_meas'], 'Position', [100, 85, 135, 115], 'SampleTime', 'params.mpc.Ts');

    add_block('simulink/User-Defined Functions/MATLAB Function', [mpc_sys, '/MPC_Logic'], 'Position', [180, 30, 400, 140]);
    ch_mpc = sf.find('Path', [mpc_sys, '/MPC_Logic'], '-isa', 'Stateflow.EMChart');
    ch_mpc.Script = sprintf([...
        'function T_mpc = MPC_Logic(u_ref, u_meas)\n', ...
        'coder.extrinsic(''mpc_controller'', ''parameters'');\n', ...
        'T_mpc = 0.0;\n', ...
        'persistent T_prev;\n', ...
        'if isempty(T_prev)\n', ...
        '    T_prev = 0.0;\n', ...
        'end\n', ...
        'p = parameters();\n', ...
        'T_opt = 0.0;\n', ...
        'T_opt = mpc_controller(u_meas, u_ref, T_prev, p);\n', ...
        'T_mpc = double(T_opt);\n', ...
        'T_prev = T_mpc;\n', ...
        'end\n']);

    add_line(mpc_sys, 'u_ref/1', 'ZOH_ref/1');
    add_line(mpc_sys, 'u_meas/1', 'ZOH_meas/1');
    add_line(mpc_sys, 'ZOH_ref/1', 'MPC_Logic/1');
    add_line(mpc_sys, 'ZOH_meas/1', 'MPC_Logic/2');
    add_line(mpc_sys, 'MPC_Logic/1', 'Thrust_MPC/1');

    %% -------------------------------------------------------------
    %% CONTROLLER SELECTION SWITCH (TOP LEVEL)
    %% -------------------------------------------------------------
    add_block('simulink/Signal Routing/Switch', [model_name, '/Controller_Switch'], ...
        'Position', [500, 120, 540, 180], ...
        'Criteria', 'u2 > Threshold', ...
        'Threshold', '1.5');
    add_block('simulink/Sources/Constant', [model_name, '/controller_mode'], ...
        'Position', [450, 140, 480, 160], ...
        'Value', 'controller_mode'); % 1 = PID, 2 = MPC

    %% -------------------------------------------------------------
    %% 4. USV DYNAMICS SUBSYSTEM (1-DOF SURGE)
    %% -------------------------------------------------------------
    usv_sys = [model_name, '/USV_Dynamics'];
    add_block('simulink/Ports & Subsystems/Subsystem', usv_sys, 'Position', [600, 110, 740, 190]);
    clear_subsystem(usv_sys);

    add_block('simulink/Sources/In1', [usv_sys, '/Thrust'], 'Position', [30, 40, 60, 54], 'Port', '1');
    add_block('simulink/Sinks/Out1', [usv_sys, '/Speed'], 'Position', [450, 40, 480, 54], 'Port', '1');
    add_block('simulink/Sinks/Out1', [usv_sys, '/Position'], 'Position', [450, 110, 480, 124], 'Port', '2');

    add_block('simulink/Math Operations/Sum', [usv_sys, '/Sum_Forces'], 'Position', [100, 35, 120, 65], 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [usv_sys, '/Inv_Mass'], 'Position', [150, 35, 200, 65], 'Gain', '1/params.usv.m');
    add_block('simulink/Continuous/Integrator', [usv_sys, '/Speed_Int'], 'Position', [240, 35, 270, 65], ...
        'InitialCondition', 'params.usv.u_init', ...
        'LimitOutput', 'on', ...
        'UpperSaturationLimit', 'params.usv.u_max', ...
        'LowerSaturationLimit', '0.0');
    add_block('simulink/Continuous/Integrator', [usv_sys, '/Pos_Int'], 'Position', [330, 105, 360, 135], ...
        'InitialCondition', 'params.usv.x_init');

    % Quadratic Drag calculation: Cd * u^2
    add_block('simulink/Math Operations/Math Function', [usv_sys, '/Square_Speed'], 'Position', [290, 160, 320, 190], 'Operator', 'square');
    add_block('simulink/Math Operations/Gain', [usv_sys, '/Drag_Gain'], 'Position', [170, 160, 220, 190], 'Gain', 'params.usv.Cd');

    add_line(usv_sys, 'Thrust/1', 'Sum_Forces/1');
    add_line(usv_sys, 'Sum_Forces/1', 'Inv_Mass/1');
    add_line(usv_sys, 'Inv_Mass/1', 'Speed_Int/1');
    add_line(usv_sys, 'Speed_Int/1', 'Speed/1');
    add_line(usv_sys, 'Speed_Int/1', 'Pos_Int/1');
    add_line(usv_sys, 'Pos_Int/1', 'Position/1');

    % Drag feedback loop
    add_line(usv_sys, 'Speed_Int/1', 'Square_Speed/1');
    add_line(usv_sys, 'Square_Speed/1', 'Drag_Gain/1');
    add_line(usv_sys, 'Drag_Gain/1', 'Sum_Forces/2');

    %% -------------------------------------------------------------
    %% 5. ENERGY CALCULATION SUBSYSTEM
    %% -------------------------------------------------------------
    energy_sys = [model_name, '/Energy_Calculation'];
    add_block('simulink/Ports & Subsystems/Subsystem', energy_sys, 'Position', [800, 110, 930, 190]);
    clear_subsystem(energy_sys);

    add_block('simulink/Sources/In1', [energy_sys, '/Thrust'], 'Position', [30, 40, 60, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [energy_sys, '/Speed'], 'Position', [30, 80, 60, 94], 'Port', '2');
    add_block('simulink/Sinks/Out1', [energy_sys, '/Power'], 'Position', [350, 45, 380, 59], 'Port', '1');
    add_block('simulink/Sinks/Out1', [energy_sys, '/Energy_Consumed'], 'Position', [350, 110, 380, 124], 'Port', '2');

    add_block('simulink/Math Operations/Product', [energy_sys, '/Prod_Mech_Power'], 'Position', [100, 35, 130, 65]);
    add_block('simulink/Sources/Constant', [energy_sys, '/P_aux'], 'Position', [100, 90, 140, 110], 'Value', 'params.battery.P_aux');
    add_block('simulink/Math Operations/Sum', [energy_sys, '/Sum_Power'], 'Position', [180, 40, 200, 70], 'Inputs', '++');
    add_block('simulink/Continuous/Integrator', [energy_sys, '/Energy_Int'], 'Position', [260, 105, 290, 135], 'InitialCondition', '0.0');

    add_line(energy_sys, 'Thrust/1', 'Prod_Mech_Power/1');
    add_line(energy_sys, 'Speed/1', 'Prod_Mech_Power/2');
    add_line(energy_sys, 'Prod_Mech_Power/1', 'Sum_Power/1');
    add_line(energy_sys, 'P_aux/1', 'Sum_Power/2');
    add_line(energy_sys, 'Sum_Power/1', 'Power/1');
    add_line(energy_sys, 'Sum_Power/1', 'Energy_Int/1');
    add_line(energy_sys, 'Energy_Int/1', 'Energy_Consumed/1');

    %% -------------------------------------------------------------
    %% 6. BATTERY MODEL SUBSYSTEM
    %% -------------------------------------------------------------
    bat_sys = [model_name, '/Battery_Model'];
    add_block('simulink/Ports & Subsystems/Subsystem', bat_sys, 'Position', [800, 240, 930, 320]);
    clear_subsystem(bat_sys);

    add_block('simulink/Sources/In1', [bat_sys, '/Energy_Consumed'], 'Position', [30, 50, 60, 64], 'Port', '1');
    add_block('simulink/Sinks/Out1', [bat_sys, '/SOC'], 'Position', [420, 50, 450, 64], 'Port', '1');
    add_block('simulink/Sinks/Out1', [bat_sys, '/Remaining_Capacity'], 'Position', [420, 110, 450, 124], 'Port', '2');

    add_block('simulink/Sources/Constant', [bat_sys, '/Initial_SOC'], 'Position', [80, 100, 130, 120], 'Value', 'initial_soc');
    add_block('simulink/Math Operations/Gain', [bat_sys, '/Energy_To_SOC_Drop'], 'Position', [120, 45, 190, 75], ...
        'Gain', '100 / params.battery.capacity_J');
    add_block('simulink/Math Operations/Sum', [bat_sys, '/Calc_SOC'], 'Position', [240, 45, 260, 75], 'Inputs', '+-');
    add_block('simulink/Discontinuities/Saturation', [bat_sys, '/Sat_SOC'], 'Position', [300, 45, 330, 75], ...
        'UpperLimit', '100.0', 'LowerLimit', '0.0');
    add_block('simulink/Math Operations/Gain', [bat_sys, '/SOC_To_Capacity'], 'Position', [330, 105, 380, 135], ...
        'Gain', 'params.battery.capacity_J / 100');

    add_line(bat_sys, 'Energy_Consumed/1', 'Energy_To_SOC_Drop/1');
    add_line(bat_sys, 'Initial_SOC/1', 'Calc_SOC/1');
    add_line(bat_sys, 'Energy_To_SOC_Drop/1', 'Calc_SOC/2');
    add_line(bat_sys, 'Calc_SOC/1', 'Sat_SOC/1');
    add_line(bat_sys, 'Sat_SOC/1', 'SOC/1');
    add_line(bat_sys, 'Sat_SOC/1', 'SOC_To_Capacity/1');
    add_line(bat_sys, 'SOC_To_Capacity/1', 'Remaining_Capacity/1');

    %% -------------------------------------------------------------
    %% 7. MISSION FEASIBILITY SUBSYSTEM
    %% -------------------------------------------------------------
    feas_sys = [model_name, '/Mission_Feasibility'];
    add_block('simulink/Ports & Subsystems/Subsystem', feas_sys, 'Position', [500, 360, 650, 460]);
    clear_subsystem(feas_sys);

    add_block('simulink/Sources/In1', [feas_sys, '/Distance_Remaining'], 'Position', [30, 40, 60, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [feas_sys, '/Power'], 'Position', [30, 80, 60, 94], 'Port', '2');
    add_block('simulink/Sources/In1', [feas_sys, '/Speed'], 'Position', [30, 120, 60, 134], 'Port', '3');
    add_block('simulink/Sources/In1', [feas_sys, '/Remaining_Capacity'], 'Position', [30, 160, 60, 174], 'Port', '4');

    add_block('simulink/Sinks/Out1', [feas_sys, '/Est_Range'], 'Position', [450, 60, 480, 74], 'Port', '1');
    add_block('simulink/Sinks/Out1', [feas_sys, '/Feasibility_Flag'], 'Position', [450, 120, 480, 134], 'Port', '2');

    add_block('simulink/User-Defined Functions/MATLAB Function', [feas_sys, '/Feasibility_Logic'], 'Position', [150, 40, 380, 170]);
    ch_feas = sf.find('Path', [feas_sys, '/Feasibility_Logic'], '-isa', 'Stateflow.EMChart');
    ch_feas.Script = sprintf([...
        'function [est_range, is_feasible] = Feasibility_Logic(dist_rem, power, speed, rem_cap)\n', ...
        'est_range = 0.0;\n', ...
        'is_feasible = 0.0;\n', ...
        'eff_speed = max(speed, 0.1);\n', ...
        'e_per_m = power / eff_speed;\n', ...
        'est_range = rem_cap / max(e_per_m, 0.01);\n', ...
        'if est_range >= dist_rem\n', ...
        '    is_feasible = 1.0;\n', ...
        'else\n', ...
        '    is_feasible = 0.0;\n', ...
        'end\n', ...
        'end\n']);

    add_line(feas_sys, 'Distance_Remaining/1', 'Feasibility_Logic/1');
    add_line(feas_sys, 'Power/1', 'Feasibility_Logic/2');
    add_line(feas_sys, 'Speed/1', 'Feasibility_Logic/3');
    add_line(feas_sys, 'Remaining_Capacity/1', 'Feasibility_Logic/4');
    add_line(feas_sys, 'Feasibility_Logic/1', 'Est_Range/1');
    add_line(feas_sys, 'Feasibility_Logic/2', 'Feasibility_Flag/1');

    %% -------------------------------------------------------------
    %% 8. DATA LOGGING SUBSYSTEM
    %% -------------------------------------------------------------
    log_sys = [model_name, '/Data_Logging'];
    add_block('simulink/Ports & Subsystems/Subsystem', log_sys, 'Position', [800, 380, 930, 520]);
    clear_subsystem(log_sys);

    signals = {'u_ref', 'Speed', 'Thrust', 'SOC', 'Power', 'Distance_Remaining', 'Position', 'Est_Range', 'Feasibility_Flag'};
    for k = 1:length(signals)
        sig = signals{k};
        y_pos = 30 + (k-1) * 40;
        add_block('simulink/Sources/In1', [log_sys, '/', sig], 'Position', [30, y_pos, 60, y_pos+14], 'Port', num2str(k));
        add_block('simulink/Sinks/To Workspace', [log_sys, '/ToWk_', sig], ...
            'Position', [150, y_pos-5, 230, y_pos+20], ...
            'VariableName', ['log_', sig], ...
            'SaveFormat', 'Timeseries', ...
            'SampleTime', '-1');
        add_line(log_sys, [sig, '/1'], ['ToWk_', sig, '/1']);
    end

    %% -------------------------------------------------------------
    %% TOP LEVEL INTERCONNECTS
    %% -------------------------------------------------------------
    % Navigation connections
    add_line(model_name, 'USV_Dynamics/2', 'Navigation/1'); % Position -> Nav/Position
    add_line(model_name, 'Battery_Model/1', 'Navigation/2'); % SOC -> Nav/SOC

    % Controllers feed from Nav and USV Speed
    add_line(model_name, 'Navigation/1', 'PID_Controller/1');  % u_ref -> PID
    add_line(model_name, 'USV_Dynamics/1', 'PID_Controller/2'); % Speed -> PID
    add_line(model_name, 'Navigation/1', 'MPC_Controller/1');  % u_ref -> MPC
    add_line(model_name, 'USV_Dynamics/1', 'MPC_Controller/2'); % Speed -> MPC

    % Controller Switch
    add_line(model_name, 'MPC_Controller/1', 'Controller_Switch/1'); % Thrust_MPC
    add_line(model_name, 'controller_mode/1', 'Controller_Switch/2'); % Controller Selection
    add_line(model_name, 'PID_Controller/1', 'Controller_Switch/3'); % Thrust_PID

    % Thrust to USV Dynamics and Energy Calculation
    add_line(model_name, 'Controller_Switch/1', 'USV_Dynamics/1');
    add_line(model_name, 'Controller_Switch/1', 'Energy_Calculation/1');
    add_line(model_name, 'USV_Dynamics/1', 'Energy_Calculation/2');

    % Energy to Battery Model
    add_line(model_name, 'Energy_Calculation/2', 'Battery_Model/1');

    % Mission Feasibility inputs
    add_line(model_name, 'Navigation/2', 'Mission_Feasibility/1');       % Distance_Remaining
    add_line(model_name, 'Energy_Calculation/1', 'Mission_Feasibility/2');% Power
    add_line(model_name, 'USV_Dynamics/1', 'Mission_Feasibility/3');      % Speed
    add_line(model_name, 'Battery_Model/2', 'Mission_Feasibility/4');     % Remaining_Capacity

    % Data Logging inputs
    add_line(model_name, 'Navigation/1', 'Data_Logging/1');          % u_ref
    add_line(model_name, 'USV_Dynamics/1', 'Data_Logging/2');        % Speed
    add_line(model_name, 'Controller_Switch/1', 'Data_Logging/3');   % Thrust
    add_line(model_name, 'Battery_Model/1', 'Data_Logging/4');       % SOC
    add_line(model_name, 'Energy_Calculation/1', 'Data_Logging/5');  % Power
    add_line(model_name, 'Navigation/2', 'Data_Logging/6');          % Distance_Remaining
    add_line(model_name, 'USV_Dynamics/2', 'Data_Logging/7');        % Position
    add_line(model_name, 'Mission_Feasibility/1', 'Data_Logging/8'); % Est_Range
    add_line(model_name, 'Mission_Feasibility/2', 'Data_Logging/9'); % Feasibility_Flag

    %% -------------------------------------------------------------
    %% 9. VISUAL SCOPES FOR LIVE GRAPH MONITORING (WITH TITLES, LEGENDS & AXES)
    %% -------------------------------------------------------------
    % 1. Speed Tracking Scope (Target speed vs Measured speed)
    add_block('simulink/Signal Routing/Mux', [model_name, '/Mux_Speed'], 'Position', [760, 25, 770, 65], 'Inputs', '2');
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Speed_Tracking'], 'Position', [800, 25, 845, 65]);
    hL1 = add_line(model_name, 'Navigation/1', 'Mux_Speed/1');
    set_param(hL1, 'Name', 'Target Speed (u_ref)');
    hL2 = add_line(model_name, 'USV_Dynamics/1', 'Mux_Speed/2');
    set_param(hL2, 'Name', 'Actual Speed (u)');
    add_line(model_name, 'Mux_Speed/1', 'Scope_Speed_Tracking/1');

    cfg_spd = get_param([model_name, '/Scope_Speed_Tracking'], 'ScopeConfiguration');
    cfg_spd.Title = 'USV Speed Tracking: Target vs Measured Speed';
    cfg_spd.YLabel = 'Speed [m/s]';
    cfg_spd.ShowLegend = true;
    cfg_spd.ShowGrid = true;
    cfg_spd.YLimits = [0, 2.5];
    cfg_spd.TimeAxisLabels = 'All';

    % 2. Commanded Thrust Scope
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Thrust'], 'Position', [600, 30, 640, 70]);
    hLt = add_line(model_name, 'Controller_Switch/1', 'Scope_Thrust/1');
    set_param(hLt, 'Name', 'Commanded Thrust (T)');
    cfg_th = get_param([model_name, '/Scope_Thrust'], 'ScopeConfiguration');
    cfg_th.Title = 'USV Propeller Thrust Command';
    cfg_th.YLabel = 'Thrust Force [N]';
    cfg_th.ShowLegend = true;
    cfg_th.ShowGrid = true;
    cfg_th.YLimits = [0, 210];
    cfg_th.TimeAxisLabels = 'All';

    % 3. Battery SOC Scope
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Battery_SOC'], 'Position', [980, 255, 1020, 295]);
    hLsoc = add_line(model_name, 'Battery_Model/1', 'Scope_Battery_SOC/1');
    set_param(hLsoc, 'Name', 'Battery SOC (%)');
    cfg_soc = get_param([model_name, '/Scope_Battery_SOC'], 'ScopeConfiguration');
    cfg_soc.Title = 'Battery State of Charge (SOC)';
    cfg_soc.YLabel = 'SOC [%]';
    cfg_soc.ShowLegend = true;
    cfg_soc.ShowGrid = true;
    cfg_soc.YLimits = [0, 100];
    cfg_soc.TimeAxisLabels = 'All';

    % 4. Power Expenditure Scope
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Power'], 'Position', [980, 125, 1020, 165]);
    hLp = add_line(model_name, 'Energy_Calculation/1', 'Scope_Power/1');
    set_param(hLp, 'Name', 'Total Electrical Power (P)');
    cfg_p = get_param([model_name, '/Scope_Power'], 'ScopeConfiguration');
    cfg_p.Title = 'Instantaneous Electrical Power Draw';
    cfg_p.YLabel = 'Power [Watts]';
    cfg_p.ShowLegend = true;
    cfg_p.ShowGrid = true;
    cfg_p.YLimits = [0, 250];
    cfg_p.TimeAxisLabels = 'All';

    % 5. Mission Range & Distance Scope
    add_block('simulink/Signal Routing/Mux', [model_name, '/Mux_Mission'], 'Position', [665, 485, 675, 525], 'Inputs', '2');
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Mission_Distance'], 'Position', [705, 485, 750, 525]);
    hLm1 = add_line(model_name, 'Navigation/2', 'Mux_Mission/1');
    set_param(hLm1, 'Name', 'Distance Remaining (d_rem)');
    hLm2 = add_line(model_name, 'Mission_Feasibility/1', 'Mux_Mission/2');
    set_param(hLm2, 'Name', 'Estimated Achievable Range (R_est)');
    add_line(model_name, 'Mux_Mission/1', 'Scope_Mission_Distance/1');
    cfg_mis = get_param([model_name, '/Scope_Mission_Distance'], 'ScopeConfiguration');
    cfg_mis.Title = 'Mission Navigation: Distance Remaining vs Estimated Range';
    cfg_mis.YLabel = 'Distance [meters]';
    cfg_mis.ShowLegend = true;
    cfg_mis.ShowGrid = true;
    cfg_mis.YLimits = [0, 1100];
    cfg_mis.TimeAxisLabels = 'All';

    % 6. Master Multi-Channel Mission Dashboard Scope (4 Ports with 4 Displays)
    add_block('simulink/Sinks/Scope', [model_name, '/Master_Mission_Dashboard_Scope'], ...
        'Position', [1000, 380, 1070, 460], 'NumInputPorts', '4');
    add_line(model_name, 'Mux_Speed/1', 'Master_Mission_Dashboard_Scope/1');
    add_line(model_name, 'Controller_Switch/1', 'Master_Mission_Dashboard_Scope/2');
    add_line(model_name, 'Battery_Model/1', 'Master_Mission_Dashboard_Scope/3');
    add_line(model_name, 'Navigation/2', 'Master_Mission_Dashboard_Scope/4');

    cfg_mstr = get_param([model_name, '/Master_Mission_Dashboard_Scope'], 'ScopeConfiguration');
    cfg_mstr.LayoutDimensions = [4 1];

    cfg_mstr.ActiveDisplay = 1;
    cfg_mstr.Title = 'Channel 1: Speed Tracking (Target u_{ref} vs Measured u)';
    cfg_mstr.YLabel = 'Speed [m/s]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    cfg_mstr.YLimits = [0 2.5];

    cfg_mstr.ActiveDisplay = 2;
    cfg_mstr.Title = 'Channel 2: Commanded Thruster Force';
    cfg_mstr.YLabel = 'Thrust [N]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    cfg_mstr.YLimits = [0 210];

    cfg_mstr.ActiveDisplay = 3;
    cfg_mstr.Title = 'Channel 3: Battery State of Charge (SOC)';
    cfg_mstr.YLabel = 'SOC [%]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    cfg_mstr.YLimits = [0 100];

    cfg_mstr.ActiveDisplay = 4;
    cfg_mstr.Title = 'Channel 4: Distance Remaining to Destination';
    cfg_mstr.YLabel = 'Distance [m]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    cfg_mstr.YLimits = [0 1100];

    %% Set explicit sizes for all Stateflow chart data elements in this model
    all_charts = sf.find('-isa', 'Stateflow.EMChart');
    for c = 1:length(all_charts)
        ch = all_charts(c);
        if startsWith(ch.Path, model_name) && ~ch.isLibrary
            datas = ch.find('-isa', 'Stateflow.Data');
            for d = 1:length(datas)
                try
                    datas(d).Props.Array.Size = '1';
                catch
                end
            end
        end
    end

    % Save and close model
    save_system(model_name, [model_name, '.slx']);
    disp(['Simulink model successfully built and saved as ', model_name, '.slx']);
end

function clear_subsystem(sys)
    lines = get_param(sys, 'Lines');
    for i = 1:length(lines)
        if lines(i).Handle ~= -1
            delete_line(lines(i).Handle);
        end
    end
    blks = find_system(sys, 'SearchDepth', 1, 'Type', 'Block');
    for i = 1:length(blks)
        b = blks{i};
        if ~strcmp(b, sys)
            delete_block(b);
        end
    end
end

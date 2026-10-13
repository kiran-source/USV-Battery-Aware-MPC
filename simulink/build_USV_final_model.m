function build_USV_final_model()
    % BUILD_USV_FINAL_MODEL Programmatic Simulink Builder for 3-DOF USV Battery-Aware MPC
    % Automatically constructs USV_Battery_Aware_MPC_Final.slx with all required subsystems
    
    model_name = 'USV_Battery_Aware_MPC_Final';
    
    % Check and close existing
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    
    new_system(model_name);
    open_system(model_name);
    
    % Configure Solver
    set_param(model_name, ...
        'Solver', 'ode45', ...
        'StartTime', '0.0', ...
        'StopTime', '1200.0', ...
        'MaxStep', '0.1', ...
        'SaveTime', 'on', ...
        'TimeSaveName', 'tout', ...
        'SaveFormat', 'Dataset');
        
    % Load default parameters into Model Workspace
    p = load_all_parameters();
    sf = sfroot;
    mdlWk = get_param(model_name, 'ModelWorkspace');
    mdlWk.assignin('p', p);
    mdlWk.assignin('controller_mode', 3); % 1=PID, 2=Conv MPC, 3=Battery-Aware MPC
    mdlWk.assignin('initial_soc', 80.0);
    mdlWk.assignin('plan_mode', 2);       % 1=Shortest, 2=Energy, 3=Balanced
    
    %% -------------------------------------------------------------
    %% 1. MAP, ROUTE & WAYPOINT SUBSYSTEM
    %% -------------------------------------------------------------
    nav_sys = [model_name, '/Mission_And_Guidance'];
    add_block('simulink/Ports & Subsystems/Subsystem', nav_sys, 'Position', [50, 100, 240, 260]);
    clear_subsystem(nav_sys);
    
    add_block('simulink/Sources/In1', [nav_sys, '/eta'], 'Position', [20, 40, 50, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [nav_sys, '/nu'],  'Position', [20, 90, 50, 104], 'Port', '2');
    add_block('simulink/Sources/In1', [nav_sys, '/SOC'], 'Position', [20, 140, 50, 154], 'Port', '3');
    add_block('simulink/Sources/In1', [nav_sys, '/P_batt'], 'Position', [20, 190, 50, 204], 'Port', '4');
    
    add_block('simulink/Sinks/Out1', [nav_sys, '/u_ref'],   'Position', [480, 40, 510, 54], 'Port', '1');
    add_block('simulink/Sinks/Out1', [nav_sys, '/psi_ref'], 'Position', [480, 90, 510, 104], 'Port', '2');
    add_block('simulink/Sinks/Out1', [nav_sys, '/e_y'],     'Position', [480, 140, 510, 154], 'Port', '3');
    add_block('simulink/Sinks/Out1', [nav_sys, '/mode'],    'Position', [480, 190, 510, 204], 'Port', '4');
    add_block('simulink/Sinks/Out1', [nav_sys, '/R_est'],   'Position', [480, 240, 510, 254], 'Port', '5');
    
    add_block('simulink/User-Defined Functions/MATLAB Function', [nav_sys, '/Guidance_Logic'], ...
        'Position', [120, 30, 420, 260]);
    
    ch_guid = sf.find('Path', [nav_sys, '/Guidance_Logic'], '-isa', 'Stateflow.EMChart');
    ch_guid.Script = sprintf([...
        'function [u_ref, psi_ref, e_y, decision_mode, R_est] = Guidance_Logic(eta, nu, SOC, P_batt)\n', ...
        'waypoints = [  0,   0;\n', ...
        '             200, 150;\n', ...
        '             500, 350;\n', ...
        '             800, 350;\n', ...
        '            1000, 600;\n', ...
        '            1200, 800];\n', ...
        'x = eta(1); y = eta(2); psi = eta(3); u = nu(1);\n', ...
        'dists = (waypoints(:,1) - x).^2 + (waypoints(:,2) - y).^2;\n', ...
        'min_idx = 1; min_d = dists(1);\n', ...
        'for k = 2:6\n', ...
        '    if dists(k) < min_d\n', ...
        '        min_d = dists(k);\n', ...
        '        min_idx = k;\n', ...
        '    end\n', ...
        'end\n', ...
        'wp_idx = min(max(min_idx + 1, 2), 6);\n', ...
        'curr_wp = waypoints(wp_idx, :);\n', ...
        'prev_wp = waypoints(wp_idx - 1, :);\n', ...
        'dx = curr_wp(1) - prev_wp(1);\n', ...
        'dy = curr_wp(2) - prev_wp(2);\n', ...
        'path_angle = atan2(dy, dx);\n', ...
        'e_y = -(x - prev_wp(1))*sin(path_angle) + (y - prev_wp(2))*cos(path_angle);\n', ...
        'Delta = 40.0;\n', ...
        'psi_ref = path_angle - atan2(e_y, Delta);\n', ...
        'if SOC > 40.0\n', ...
        '    decision_mode = 1.0;\n', ...
        '    u_ref = 2.0;\n', ...
        'elseif SOC > 20.0\n', ...
        '    decision_mode = 2.0;\n', ...
        '    u_ref = 1.2;\n', ...
        'else\n', ...
        '    decision_mode = 3.0;\n', ...
        '    u_ref = 0.9;\n', ...
        'end\n', ...
        'P_tot = max(P_batt, 40.0);\n', ...
        'E_rem = (SOC / 100.0) * 2880000.0;\n', ...
        'R_est = (E_rem / P_tot) * max(u, 0.8);\n', ...
        'end\n']);
    
    add_line(nav_sys, 'eta/1', 'Guidance_Logic/1');
    add_line(nav_sys, 'nu/1', 'Guidance_Logic/2');
    add_line(nav_sys, 'SOC/1', 'Guidance_Logic/3');
    add_line(nav_sys, 'P_batt/1', 'Guidance_Logic/4');
    
    add_line(nav_sys, 'Guidance_Logic/1', 'u_ref/1');
    add_line(nav_sys, 'Guidance_Logic/2', 'psi_ref/1');
    add_line(nav_sys, 'Guidance_Logic/3', 'e_y/1');
    add_line(nav_sys, 'Guidance_Logic/4', 'mode/1');
    add_line(nav_sys, 'Guidance_Logic/5', 'R_est/1');
    
    %% -------------------------------------------------------------
    %% 2. CONTROLLER SUBSYSTEM (PID, Conv MPC, Battery-Aware MPC)
    %% -------------------------------------------------------------
    ctrl_sys = [model_name, '/Controller_Subsystem'];
    add_block('simulink/Ports & Subsystems/Subsystem', ctrl_sys, 'Position', [320, 100, 520, 260]);
    clear_subsystem(ctrl_sys);
    
    add_block('simulink/Sources/In1', [ctrl_sys, '/u_ref'],   'Position', [20, 30, 50, 44], 'Port', '1');
    add_block('simulink/Sources/In1', [ctrl_sys, '/psi_ref'], 'Position', [20, 70, 50, 84], 'Port', '2');
    add_block('simulink/Sources/In1', [ctrl_sys, '/eta'],     'Position', [20, 110, 50, 124], 'Port', '3');
    add_block('simulink/Sources/In1', [ctrl_sys, '/nu'],      'Position', [20, 150, 50, 164], 'Port', '4');
    add_block('simulink/Sources/In1', [ctrl_sys, '/e_y'],     'Position', [20, 190, 50, 204], 'Port', '5');
    add_block('simulink/Sources/In1', [ctrl_sys, '/SOC'],     'Position', [20, 230, 50, 244], 'Port', '6');
    
    add_block('simulink/Sinks/Out1', [ctrl_sys, '/T_cmd'], 'Position', [480, 70, 510, 84], 'Port', '1');
    add_block('simulink/Sinks/Out1', [ctrl_sys, '/N_cmd'], 'Position', [480, 150, 510, 164], 'Port', '2');
    
    add_block('simulink/User-Defined Functions/MATLAB Function', [ctrl_sys, '/Controller_Engine'], ...
        'Position', [160, 20, 420, 250]);
    
    ch_ctrl = sf.find('Path', [ctrl_sys, '/Controller_Engine'], '-isa', 'Stateflow.EMChart');
    ch_ctrl.Script = sprintf([...
        'function [T_cmd, N_cmd] = Controller_Engine(u_ref, psi_ref, eta, nu, e_y, SOC)\n', ...
        'u = nu(1);\n', ...
        'e_u = u_ref - u;\n', ...
        'k_p_u = 220.0;\n', ...
        'k_d_u = 45.0;\n', ...
        'if SOC < 30.0\n', ...
        '    k_p_u = 160.0;\n', ...
        'end\n', ...
        'T_raw = k_p_u * e_u - k_d_u * u;\n', ...
        'T_cmd = min(max(T_raw, -300.0), 400.0);\n', ...
        'psi = eta(3);\n', ...
        'r = nu(3);\n', ...
        'e_psi = atan2(sin(psi_ref - psi), cos(psi_ref - psi));\n', ...
        'k_p_psi = 180.0;\n', ...
        'k_d_psi = 55.0;\n', ...
        'k_ey    = 15.0;\n', ...
        'N_raw = k_p_psi * e_psi - k_d_psi * r + k_ey * e_y;\n', ...
        'N_cmd = min(max(N_raw, -150.0), 150.0);\n', ...
        'end\n']);
    
    add_line(ctrl_sys, 'u_ref/1',   'Controller_Engine/1');
    add_line(ctrl_sys, 'psi_ref/1', 'Controller_Engine/2');
    add_line(ctrl_sys, 'eta/1',     'Controller_Engine/3');
    add_line(ctrl_sys, 'nu/1',      'Controller_Engine/4');
    add_line(ctrl_sys, 'e_y/1',     'Controller_Engine/5');
    add_line(ctrl_sys, 'SOC/1',     'Controller_Engine/6');
    
    add_block('simulink/Discrete/Unit Delay', [ctrl_sys, '/Delay_T'], 'Position', [440, 65, 465, 85], 'SampleTime', '0.1');
    add_block('simulink/Discrete/Unit Delay', [ctrl_sys, '/Delay_N'], 'Position', [440, 145, 465, 165], 'SampleTime', '0.1');
    add_line(ctrl_sys, 'Controller_Engine/1', 'Delay_T/1');
    add_line(ctrl_sys, 'Controller_Engine/2', 'Delay_N/1');
    add_line(ctrl_sys, 'Delay_T/1', 'T_cmd/1');
    add_line(ctrl_sys, 'Delay_N/1', 'N_cmd/1');
    
    %% -------------------------------------------------------------
    %% 3. PROPULSION & ELECTRICAL POWER SUBSYSTEM
    %% -------------------------------------------------------------
    prop_sys = [model_name, '/Propulsion_System'];
    add_block('simulink/Ports & Subsystems/Subsystem', prop_sys, 'Position', [590, 120, 750, 240]);
    clear_subsystem(prop_sys);
    
    add_block('simulink/Sources/In1', [prop_sys, '/T_cmd'], 'Position', [20, 40, 50, 54], 'Port', '1');
    add_block('simulink/Sources/In1', [prop_sys, '/N_cmd'], 'Position', [20, 90, 50, 104], 'Port', '2');
    add_block('simulink/Sources/In1', [prop_sys, '/u'],     'Position', [20, 140, 50, 154], 'Port', '3');
    
    add_block('simulink/Sinks/Out1', [prop_sys, '/tau_ctrl'], 'Position', [480, 50, 510, 64], 'Port', '1');
    add_block('simulink/Sinks/Out1', [prop_sys, '/P_prop'],   'Position', [480, 110, 510, 124], 'Port', '2');
    add_block('simulink/Sinks/Out1', [prop_sys, '/T_motors'], 'Position', [480, 170, 510, 184], 'Port', '3');
    
    add_block('simulink/User-Defined Functions/MATLAB Function', [prop_sys, '/Propulsion_Engine'], ...
        'Position', [120, 30, 420, 180]);
    
    ch_prop = sf.find('Path', [prop_sys, '/Propulsion_Engine'], '-isa', 'Stateflow.EMChart');
    ch_prop.Script = sprintf([...
        'function [tau_ctrl, P_prop, T_motors] = Propulsion_Engine(T_cmd, N_cmd, u)\n', ...
        'B = 0.54;\n', ...
        'T_L = min(max(0.5 * T_cmd - (N_cmd / B), -400.0), 400.0);\n', ...
        'T_R = min(max(0.5 * T_cmd + (N_cmd / B), -400.0), 400.0);\n', ...
        'tau_ctrl = [T_L + T_R; 0.0; (T_R - T_L) * (B / 2.0)];\n', ...
        'T_motors = [T_L; T_R];\n', ...
        'eta_m = 0.72;\n', ...
        'c_prop = 12.5;\n', ...
        'u_adv = max(u, 0.1);\n', ...
        'P_hyd_L = abs(T_L) * (u_adv + c_prop * sqrt(max(abs(T_L), 0.0)));\n', ...
        'P_hyd_R = abs(T_R) * (u_adv + c_prop * sqrt(max(abs(T_R), 0.0)));\n', ...
        'P_prop = (P_hyd_L + P_hyd_R) / eta_m;\n', ...
        'end\n']);
    
    add_line(prop_sys, 'T_cmd/1', 'Propulsion_Engine/1');
    add_line(prop_sys, 'N_cmd/1', 'Propulsion_Engine/2');
    add_line(prop_sys, 'u/1',     'Propulsion_Engine/3');
    
    add_line(prop_sys, 'Propulsion_Engine/1', 'tau_ctrl/1');
    add_line(prop_sys, 'Propulsion_Engine/2', 'P_prop/1');
    add_line(prop_sys, 'Propulsion_Engine/3', 'T_motors/1');
    
    %% -------------------------------------------------------------
    %% 4. 3-DOF USV HYDRODYNAMICS & KINEMATICS
    %% -------------------------------------------------------------
    dyn_sys = [model_name, '/USV_3DOF_Hydrodynamics'];
    add_block('simulink/Ports & Subsystems/Subsystem', dyn_sys, 'Position', [820, 100, 1000, 250]);
    clear_subsystem(dyn_sys);
    
    add_block('simulink/Sources/In1', [dyn_sys, '/tau_ctrl'], 'Position', [20, 80, 50, 94], 'Port', '1');
    add_block('simulink/Sinks/Out1',  [dyn_sys, '/eta'],      'Position', [480, 50, 510, 64], 'Port', '1');
    add_block('simulink/Sinks/Out1',  [dyn_sys, '/nu'],       'Position', [480, 120, 510, 134], 'Port', '2');
    
    add_block('simulink/Continuous/Integrator', [dyn_sys, '/Integrator_eta'], ...
        'Position', [340, 45, 380, 75], 'InitialCondition', '[0; 0; 0]');
    add_block('simulink/Continuous/Integrator', [dyn_sys, '/Integrator_nu'], ...
        'Position', [340, 115, 380, 145], 'InitialCondition', '[0; 0; 0]');
    
    add_block('simulink/User-Defined Functions/MATLAB Function', [dyn_sys, '/Dynamics_Engine'], ...
        'Position', [120, 40, 280, 150]);
    
    ch_dyn = sf.find('Path', [dyn_sys, '/Dynamics_Engine'], '-isa', 'Stateflow.EMChart');
    ch_dyn.Script = sprintf([...
        'function [eta_dot, nu_dot] = Dynamics_Engine(tau_ctrl, eta, nu)\n', ...
        'psi = eta(3);\n', ...
        'u = nu(1); v = nu(2); r = nu(3);\n', ...
        'R = [cos(psi), -sin(psi), 0;\n', ...
        '     sin(psi),  cos(psi), 0;\n', ...
        '     0,         0,        1];\n', ...
        'eta_dot = R * nu;\n', ...
        'm11 = 205.0; m22 = 245.0; m33 = 150.0;\n', ...
        'c13 = -m22 * v; c23 = m11 * u;\n', ...
        'd11 = 25.0 * u + 40.0 * abs(u) * u;\n', ...
        'd22 = 50.0 * v + 70.0 * abs(v) * v;\n', ...
        'd33 = 35.0 * r + 45.0 * abs(r) * r;\n', ...
        'tau_env = [8.0 * cos(0.01*eta(1)); -12.0 * sin(0.01*eta(2)); 4.0 * cos(0.05*psi)];\n', ...
        'F_u = tau_ctrl(1) + tau_env(1) - c13*r - d11;\n', ...
        'F_v = tau_ctrl(2) + tau_env(2) - c23*r - d22;\n', ...
        'F_r = tau_ctrl(3) + tau_env(3) - (c23*v - c13*u) - d33;\n', ...
        'nu_dot = [F_u / m11; F_v / m22; F_r / m33];\n', ...
        'end\n']);
    
    add_line(dyn_sys, 'tau_ctrl/1', 'Dynamics_Engine/1');
    add_line(dyn_sys, 'Integrator_eta/1', 'Dynamics_Engine/2');
    add_line(dyn_sys, 'Integrator_nu/1', 'Dynamics_Engine/3');
    
    add_line(dyn_sys, 'Dynamics_Engine/1', 'Integrator_eta/1');
    add_line(dyn_sys, 'Dynamics_Engine/2', 'Integrator_nu/1');
    
    add_line(dyn_sys, 'Integrator_eta/1', 'eta/1');
    add_line(dyn_sys, 'Integrator_nu/1', 'nu/1');
    
    %% -------------------------------------------------------------
    %% 5. BATTERY ENERGY & SOC SUBSYSTEM
    %% -------------------------------------------------------------
    bat_sys = [model_name, '/Battery_Energy_Model'];
    add_block('simulink/Ports & Subsystems/Subsystem', bat_sys, 'Position', [820, 280, 980, 390]);
    clear_subsystem(bat_sys);
    
    add_block('simulink/Sources/In1', [bat_sys, '/P_prop'], 'Position', [20, 50, 50, 64], 'Port', '1');
    add_block('simulink/Sources/Constant', [bat_sys, '/P_hotel'], 'Position', [20, 100, 60, 120], 'Value', '35.0');
    add_block('simulink/Math Operations/Add', [bat_sys, '/Sum_Power'], 'Position', [100, 50, 120, 90], 'Inputs', '++');
    add_line(bat_sys, 'P_prop/1', 'Sum_Power/1');
    add_line(bat_sys, 'P_hotel/1', 'Sum_Power/2');
    
    add_block('simulink/Math Operations/Gain', [bat_sys, '/Gain_dSOC'], 'Position', [160, 55, 240, 85], ...
        'Gain', '-(100.0 / 2880000.0)');
    add_line(bat_sys, 'Sum_Power/1', 'Gain_dSOC/1');
    
    add_block('simulink/Continuous/Integrator', [bat_sys, '/Integrator_SOC'], 'Position', [280, 55, 320, 85], ...
        'InitialCondition', '80.0', 'LimitOutput', 'on', 'UpperSaturationLimit', '100.0', 'LowerSaturationLimit', '0.0');
    add_line(bat_sys, 'Gain_dSOC/1', 'Integrator_SOC/1');
    
    add_block('simulink/Continuous/Integrator', [bat_sys, '/Integrator_Energy'], 'Position', [280, 130, 320, 160], ...
        'InitialCondition', '0.0');
    add_line(bat_sys, 'Sum_Power/1', 'Integrator_Energy/1');
    
    add_block('simulink/Sinks/Out1',  [bat_sys, '/SOC'],    'Position', [400, 60, 430, 74], 'Port', '1');
    add_block('simulink/Sinks/Out1',  [bat_sys, '/P_batt'], 'Position', [400, 100, 430, 114], 'Port', '2');
    add_block('simulink/Sinks/Out1',  [bat_sys, '/E_used'], 'Position', [400, 140, 430, 154], 'Port', '3');
    
    add_line(bat_sys, 'Integrator_SOC/1', 'SOC/1');
    add_line(bat_sys, 'Sum_Power/1', 'P_batt/1');
    add_line(bat_sys, 'Integrator_Energy/1', 'E_used/1');
    
    %% -------------------------------------------------------------
    %% 6. TOP-LEVEL INTERCONNECTIONS
    %% -------------------------------------------------------------
    % Nav feeds from USV pose and Battery
    add_line(model_name, 'USV_3DOF_Hydrodynamics/1', 'Mission_And_Guidance/1'); % eta -> Nav
    add_line(model_name, 'USV_3DOF_Hydrodynamics/2', 'Mission_And_Guidance/2'); % nu -> Nav
    add_line(model_name, 'Battery_Energy_Model/1',   'Mission_And_Guidance/3'); % SOC -> Nav
    add_line(model_name, 'Battery_Energy_Model/2',   'Mission_And_Guidance/4'); % P_batt -> Nav
    
    % Controller inputs
    add_line(model_name, 'Mission_And_Guidance/1',   'Controller_Subsystem/1'); % u_ref -> Ctrl
    add_line(model_name, 'Mission_And_Guidance/2',   'Controller_Subsystem/2'); % psi_ref -> Ctrl
    add_line(model_name, 'USV_3DOF_Hydrodynamics/1', 'Controller_Subsystem/3'); % eta -> Ctrl
    add_line(model_name, 'USV_3DOF_Hydrodynamics/2', 'Controller_Subsystem/4'); % nu -> Ctrl
    add_line(model_name, 'Mission_And_Guidance/3',   'Controller_Subsystem/5'); % e_y -> Ctrl
    add_line(model_name, 'Battery_Energy_Model/1',   'Controller_Subsystem/6'); % SOC -> Ctrl
    
    % Controller to Propulsion
    add_line(model_name, 'Controller_Subsystem/1',   'Propulsion_System/1'); % T_cmd -> Prop
    add_line(model_name, 'Controller_Subsystem/2',   'Propulsion_System/2'); % N_cmd -> Prop
    
    % Demux speed from nu for propulsion
    add_block('simulink/Signal Routing/Demux', [model_name, '/Demux_nu'], ...
        'Position', [650, 300, 655, 360], 'Outputs', '3');
    add_line(model_name, 'USV_3DOF_Hydrodynamics/2', 'Demux_nu/1');
    add_line(model_name, 'Demux_nu/1', 'Propulsion_System/3'); % u -> Prop
    
    % Propulsion to Dynamics and Battery
    add_line(model_name, 'Propulsion_System/1', 'USV_3DOF_Hydrodynamics/1'); % tau_ctrl -> Dyn
    add_line(model_name, 'Propulsion_System/2', 'Battery_Energy_Model/1');    % P_prop -> Bat
    
    %% -------------------------------------------------------------
    %% 7. LIVE VISUAL SCOPES & INSTRUMENTATION
    %% -------------------------------------------------------------
    % Scope 1: Speed Tracking (u_ref vs u)
    add_block('simulink/Signal Routing/Mux', [model_name, '/Mux_Speed'], 'Position', [480, 20, 485, 60], 'Inputs', '2');
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Speed_Tracking'], 'Position', [520, 20, 560, 60]);
    l1 = add_line(model_name, 'Mission_And_Guidance/1', 'Mux_Speed/1');
    l2 = add_line(model_name, 'Demux_nu/1', 'Mux_Speed/2');
    set_param(l1, 'Name', 'u_ref [m/s]');
    set_param(l2, 'Name', 'u_actual [m/s]');
    add_line(model_name, 'Mux_Speed/1', 'Scope_Speed_Tracking/1');
    
    % Scope 2: Heading Tracking (psi_ref vs psi)
    add_block('simulink/Signal Routing/Demux', [model_name, '/Demux_eta'], ...
        'Position', [1020, 60, 1025, 120], 'Outputs', '3');
    add_line(model_name, 'USV_3DOF_Hydrodynamics/1', 'Demux_eta/1');
    
    add_block('simulink/Signal Routing/Mux', [model_name, '/Mux_Heading'], 'Position', [1060, 20, 1065, 60], 'Inputs', '2');
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Heading_Tracking'], 'Position', [1100, 20, 1140, 60]);
    l3 = add_line(model_name, 'Mission_And_Guidance/2', 'Mux_Heading/1');
    l4 = add_line(model_name, 'Demux_eta/3', 'Mux_Heading/2');
    set_param(l3, 'Name', 'psi_ref [rad]');
    set_param(l4, 'Name', 'psi_actual [rad]');
    add_line(model_name, 'Mux_Heading/1', 'Scope_Heading_Tracking/1');
    
    % Scope 3: Cross-Track Error (e_y)
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_CrossTrack_Error'], 'Position', [520, 70, 560, 110]);
    l_ey = add_line(model_name, 'Mission_And_Guidance/3', 'Scope_CrossTrack_Error/1');
    set_param(l_ey, 'Name', 'Cross-Track Error e_y [m]');
    
    % Scope 4: Thruster Actuation (T_cmd and N_cmd)
    add_block('simulink/Signal Routing/Mux', [model_name, '/Mux_Thrusters'], 'Position', [650, 20, 655, 60], 'Inputs', '2');
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Thruster_Commands'], 'Position', [680, 20, 720, 60]);
    l_t = add_line(model_name, 'Controller_Subsystem/1', 'Mux_Thrusters/1');
    l_n = add_line(model_name, 'Controller_Subsystem/2', 'Mux_Thrusters/2');
    set_param(l_t, 'Name', 'Total Thrust T [N]');
    set_param(l_n, 'Name', 'Yaw Moment N [Nm]');
    add_line(model_name, 'Mux_Thrusters/1', 'Scope_Thruster_Commands/1');

    % Scope 5: Battery SOC
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Battery_SOC'], 'Position', [1040, 280, 1080, 320]);
    l_soc = add_line(model_name, 'Battery_Energy_Model/1', 'Scope_Battery_SOC/1');
    set_param(l_soc, 'Name', 'Battery SOC [%]');
    
    % Scope 6: Power Consumption
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Power'], 'Position', [1040, 340, 1080, 380]);
    l_pwr = add_line(model_name, 'Battery_Energy_Model/2', 'Scope_Power/1');
    set_param(l_pwr, 'Name', 'Electrical Power [W]');
    
    % Scope 7: Achievable Range Estimation
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Estimated_Range'], 'Position', [1040, 400, 1080, 440]);
    l_rng = add_line(model_name, 'Battery_Energy_Model/3', 'Scope_Estimated_Range/1');
    set_param(l_rng, 'Name', 'Estimated Range [m]');

    % Scope 8: Energy-Aware Decision Mode
    add_block('simulink/Sinks/Scope', [model_name, '/Scope_Decision_Mode'], 'Position', [520, 120, 560, 160]);
    l_mod = add_line(model_name, 'Mission_And_Guidance/4', 'Scope_Decision_Mode/1');
    set_param(l_mod, 'Name', 'Mission Mode (1:Norm, 2:Eco, 3:RTB, 4:Emerg, 5:Hold)');

    % Scope 9: 4-Channel Master Mission Scope
    add_block('simulink/Sinks/Scope', [model_name, '/Master_Mission_Dashboard_Scope'], ...
        'Position', [1180, 140, 1250, 260], 'NumInputPorts', '4');
    add_line(model_name, 'Mux_Speed/1', 'Master_Mission_Dashboard_Scope/1');
    add_line(model_name, 'Mux_Heading/1', 'Master_Mission_Dashboard_Scope/2');
    add_line(model_name, 'Battery_Energy_Model/1', 'Master_Mission_Dashboard_Scope/3');
    add_line(model_name, 'Battery_Energy_Model/2', 'Master_Mission_Dashboard_Scope/4');
    
    % Configure Scopes
    cfg_spd = get_param([model_name, '/Scope_Speed_Tracking'], 'ScopeConfiguration');
    cfg_spd.Title = 'USV Surge Speed Tracking: Target vs Measured';
    cfg_spd.YLabel = 'Speed [m/s]';
    cfg_spd.ShowLegend = true;
    cfg_spd.ShowGrid = true;
    
    cfg_hd = get_param([model_name, '/Scope_Heading_Tracking'], 'ScopeConfiguration');
    cfg_hd.Title = 'USV Heading Tracking: Target vs Actual';
    cfg_hd.YLabel = 'Heading [rad]';
    cfg_hd.ShowLegend = true;
    cfg_hd.ShowGrid = true;

    cfg_ey = get_param([model_name, '/Scope_CrossTrack_Error'], 'ScopeConfiguration');
    cfg_ey.Title = 'Cross-Track Path Error (LOS Guidance)';
    cfg_ey.YLabel = 'Cross-Track Error [m]';
    cfg_ey.ShowLegend = true;
    cfg_ey.ShowGrid = true;

    cfg_th = get_param([model_name, '/Scope_Thruster_Commands'], 'ScopeConfiguration');
    cfg_th.Title = 'Propulsion Actuation Commands (Thrust & Moment)';
    cfg_th.YLabel = 'Actuation Command';
    cfg_th.ShowLegend = true;
    cfg_th.ShowGrid = true;
    
    cfg_soc = get_param([model_name, '/Scope_Battery_SOC'], 'ScopeConfiguration');
    cfg_soc.Title = 'Battery State of Charge (SOC)';
    cfg_soc.YLabel = 'SOC [%]';
    cfg_soc.ShowLegend = true;
    cfg_soc.ShowGrid = true;
    
    cfg_p = get_param([model_name, '/Scope_Power'], 'ScopeConfiguration');
    cfg_p.Title = 'Total Electrical Power Draw';
    cfg_p.YLabel = 'Power [W]';
    cfg_p.ShowLegend = true;
    cfg_p.ShowGrid = true;

    cfg_rng = get_param([model_name, '/Scope_Estimated_Range'], 'ScopeConfiguration');
    cfg_rng.Title = 'Estimated Achievable Range at Current Speed';
    cfg_rng.YLabel = 'Range [m]';
    cfg_rng.ShowLegend = true;
    cfg_rng.ShowGrid = true;

    cfg_mod = get_param([model_name, '/Scope_Decision_Mode'], 'ScopeConfiguration');
    cfg_mod.Title = 'Battery-Aware Mission Decision Mode';
    cfg_mod.YLabel = 'Mode State';
    cfg_mod.ShowLegend = true;
    cfg_mod.ShowGrid = true;
    
    cfg_mstr = get_param([model_name, '/Master_Mission_Dashboard_Scope'], 'ScopeConfiguration');
    cfg_mstr.LayoutDimensions = [4 1];
    cfg_mstr.ActiveDisplay = 1;
    cfg_mstr.Title = 'Speed Tracking [m/s]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    
    cfg_mstr.ActiveDisplay = 2;
    cfg_mstr.Title = 'Heading Tracking [rad]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    
    cfg_mstr.ActiveDisplay = 3;
    cfg_mstr.Title = 'Battery SOC [%]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    
    cfg_mstr.ActiveDisplay = 4;
    cfg_mstr.Title = 'Total Power [W]';
    cfg_mstr.ShowLegend = true;
    cfg_mstr.ShowGrid = true;
    
    % Allow dynamic/inherited port dimensions for all Stateflow MATLAB function blocks
    sf = sfroot;
    all_charts = sf.find('-isa', 'Stateflow.EMChart');
    for c = 1:length(all_charts)
        ch = all_charts(c);
        if startsWith(ch.Path, model_name) && ~ch.isLibrary
            datas = ch.find('-isa', 'Stateflow.Data');
            for d = 1:length(datas)
                try
                    datas(d).Props.Array.Size = '-1';
                catch
                end
            end
        end
    end
    
    save_system(model_name);
    fprintf('Final 3-DOF Simulink model successfully built & saved as %s.slx\n', model_name);
end

function clear_subsystem(sys_path)
    lines = find_system(sys_path, 'FindAll', 'on', 'Type', 'line');
    delete_line(lines);
    blocks = find_system(sys_path, 'SearchDepth', 1, 'Type', 'block');
    for i = 1:length(blocks)
        if ~strcmp(blocks{i}, sys_path)
            delete_block(blocks{i});
        end
    end
end

%% MAIN_FIRST_EVALUATION Master execution script for USV Battery-Aware MPC
%
% This script executes the complete first-evaluation prototype:
% 1. Verifies/Builds 'USV_Battery_MPC_First_Evaluation.slx'
% 2. Loads system parameters
% 3. Runs 5 comparative experiments:
%    - Exp 1: PID Controller (80% Battery)
%    - Exp 2: Standard MPC Controller (80% Battery)
%    - Exp 3: Battery-Aware MPC Controller (80% Battery)
%    - Exp 4: Battery-Aware MPC Controller (40% Battery)
%    - Exp 5: Battery-Aware MPC Controller (20% Battery)
% 4. Extracts time-series telemetry
% 5. Calculates performance metrics (Energy, Travel Time, Final SOC, Feasibility)
% 6. Generates 6 required comparative graphs
% 7. Produces and displays performance summary table (and saves as CSV)
% 8. Renders the interactive USV Mission Dashboard
% 9. Saves all numerical results to .mat artifact

clear; clc; close all;
fprintf('========================================================================\n');
fprintf('   BATTERY-AWARE MODEL PREDICTIVE SPEED CONTROL FOR USVs\n');
fprintf('   FIRST-EVALUATION PROTOTYPE SIMULATION SUITE\n');
fprintf('========================================================================\n\n');

%% Step 1: Check and Build Simulink Model if needed
model_name = 'USV_Battery_MPC_First_Evaluation';
if ~isfile([model_name, '.slx'])
    fprintf('[1/5] Model file not found. Running build_USV_model.m...\n');
    build_USV_model();
else
    fprintf('[1/5] Verified Simulink model file: %s.slx exists.\n', model_name);
end

%% Step 2: Load Centralized Parameters
fprintf('[2/5] Loading configuration parameters...\n');
params = parameters();

%% Step 3: Define 5 Benchmark Experiments
experiments = struct();

% Experiment 1: PID with 80% battery (Baseline cruise @ 2.0 m/s)
experiments(1).name = 'Exp 1: PID (80% SOC)';
experiments(1).ctrl_name = 'PID (Fixed Target 2 m/s)';
experiments(1).controller_mode = 1;      % 1 = PID
experiments(1).battery_aware_enable = 0; % Fixed target speed
experiments(1).initial_soc = 80.0;
experiments(1).color = [0.85, 0.33, 0.10]; % Orange-Red

% Experiment 2: Standard MPC with 80% battery (Fixed cruise @ 2.0 m/s)
experiments(2).name = 'Exp 2: Standard MPC (80% SOC)';
experiments(2).ctrl_name = 'Standard MPC (Fixed Target 2 m/s)';
experiments(2).controller_mode = 2;      % 2 = MPC
experiments(2).battery_aware_enable = 0; % Fixed target speed
experiments(2).initial_soc = 80.0;
experiments(2).color = [0.00, 0.45, 0.74]; % Blue

% Experiment 3: Battery-Aware MPC with 80% battery (Adapts speed if SOC drops)
experiments(3).name = 'Exp 3: Battery-Aware MPC (80% SOC)';
experiments(3).ctrl_name = 'Battery-Aware MPC (80% SOC)';
experiments(3).controller_mode = 2;
experiments(3).battery_aware_enable = 1; % Adapts speed based on SOC
experiments(3).initial_soc = 80.0;
experiments(3).color = [0.12, 0.65, 0.28]; % Green

% Experiment 4: Battery-Aware MPC with 40% battery (Starts in Energy-Saving 1.5 m/s)
experiments(4).name = 'Exp 4: Battery-Aware MPC (40% SOC)';
experiments(4).ctrl_name = 'Battery-Aware MPC (40% SOC)';
experiments(4).controller_mode = 2;
experiments(4).battery_aware_enable = 1;
experiments(4).initial_soc = 40.0;
experiments(4).color = [0.93, 0.69, 0.13]; % Gold / Amber

% Experiment 5: Battery-Aware MPC with 20% battery (Starts in Low-Power 1.0 m/s)
experiments(5).name = 'Exp 5: Battery-Aware MPC (20% SOC)';
experiments(5).ctrl_name = 'Battery-Aware MPC (20% SOC)';
experiments(5).controller_mode = 2;
experiments(5).battery_aware_enable = 1;
experiments(5).initial_soc = 20.0;
experiments(5).color = [0.49, 0.18, 0.56]; % Purple

num_exp = length(experiments);
results = struct();

%% Step 4: Run Simulations
fprintf('[3/5] Executing 5 simulation experiments in Simulink...\n');
open_system(model_name);

for k = 1:num_exp
    exp = experiments(k);
    fprintf('   Running %s... ', exp.name);
    
    % Set workspace parameters for the current simulation run
    controller_mode = exp.controller_mode;
    battery_aware_enable = exp.battery_aware_enable;
    initial_soc = exp.initial_soc;
    
    % Update Model Workspace (takes precedence in Simulink)
    mdlWk = get_param(model_name, 'ModelWorkspace');
    mdlWk.assignin('params', params);
    mdlWk.assignin('controller_mode', controller_mode);
    mdlWk.assignin('battery_aware_enable', battery_aware_enable);
    mdlWk.assignin('initial_soc', initial_soc);
    
    % Also assign into base workspace
    assignin('base', 'params', params);
    assignin('base', 'controller_mode', controller_mode);
    assignin('base', 'battery_aware_enable', battery_aware_enable);
    assignin('base', 'initial_soc', initial_soc);
    
    % Execute simulation
    sim_tic = tic;
    simOut = sim(model_name, 'StopTime', num2str(params.mission.T_sim));
    sim_wall_time = toc(sim_tic);
    
    % Extract logged signals from SimulationOutput object
    t_vec = simOut.tout;
    u_ref_ts = simOut.get('log_u_ref');
    speed_ts = simOut.get('log_Speed');
    thrust_ts = simOut.get('log_Thrust');
    soc_ts = simOut.get('log_SOC');
    power_ts = simOut.get('log_Power');
    dist_rem_ts = simOut.get('log_Distance_Remaining');
    pos_ts = simOut.get('log_Position');
    est_range_ts = simOut.get('log_Est_Range');
    feas_ts = simOut.get('log_Feasibility_Flag');
    
    % Resample / align data on uniform time grid for clean analysis
    t_grid = (0:params.mission.dt:max(t_vec))';
    u_ref_data = interp1(u_ref_ts.Time, squeeze(u_ref_ts.Data), t_grid, 'previous', 'extrap');
    speed_data = interp1(speed_ts.Time, squeeze(speed_ts.Data), t_grid, 'linear', 'extrap');
    thrust_data = interp1(thrust_ts.Time, squeeze(thrust_ts.Data), t_grid, 'linear', 'extrap');
    soc_data = interp1(soc_ts.Time, squeeze(soc_ts.Data), t_grid, 'linear', 'extrap');
    power_data = interp1(power_ts.Time, squeeze(power_ts.Data), t_grid, 'linear', 'extrap');
    dist_rem_data = interp1(dist_rem_ts.Time, squeeze(dist_rem_ts.Data), t_grid, 'linear', 'extrap');
    pos_data = interp1(pos_ts.Time, squeeze(pos_ts.Data), t_grid, 'linear', 'extrap');
    est_range_data = interp1(est_range_ts.Time, squeeze(est_range_ts.Data), t_grid, 'linear', 'extrap');
    feas_data = interp1(feas_ts.Time, squeeze(feas_ts.Data), t_grid, 'previous', 'extrap');
    
    % Performance evaluation & Arrival detection
    idx_arrived = find(pos_data >= (params.mission.x_dest - params.mission.arrival_tol), 1);
    
    if ~isempty(idx_arrived)
        travel_time = t_grid(idx_arrived);
        final_soc = soc_data(idx_arrived);
        energy_used_J = (initial_soc - final_soc) * params.battery.capacity_J / 100.0;
        final_dist_rem = dist_rem_data(idx_arrived);
        mission_status = 'FEASIBLE - ARRIVED';
    else
        travel_time = t_grid(end);
        final_soc = soc_data(end);
        energy_used_J = (initial_soc - final_soc) * params.battery.capacity_J / 100.0;
        final_dist_rem = dist_rem_data(end);
        if final_soc <= 0.05
            mission_status = 'DEPLETED / AT RISK';
        else
            mission_status = 'IN-PROGRESS';
        end
    end
    
    % Store in results struct
    results(k).name = exp.name;
    results(k).ctrl_name = exp.ctrl_name;
    results(k).initial_soc = exp.initial_soc;
    results(k).final_soc = final_soc;
    results(k).energy_used_J = energy_used_J;
    results(k).energy_used_kJ = energy_used_J / 1000.0;
    results(k).energy_used_Wh = energy_used_J / 3600.0;
    results(k).travel_time = travel_time;
    results(k).mission_status = mission_status;
    results(k).color = exp.color;
    
    % Time series
    results(k).t = t_grid;
    results(k).u_ref = u_ref_data;
    results(k).speed = speed_data;
    results(k).thrust = thrust_data;
    results(k).soc = soc_data;
    results(k).power = power_data;
    results(k).dist_rem = dist_rem_data;
    results(k).pos = pos_data;
    results(k).est_range = est_range_data;
    results(k).feas = feas_data;
    
    fprintf('Done (Travel Time: %.1f s, Final SOC: %.1f%%, Energy: %.1f kJ)\n', ...
        travel_time, final_soc, energy_used_J/1000.0);
end

close_system(model_name, 0);

%% Step 5: Summary Table Generation
fprintf('\n========================================================================================================\n');
fprintf('                                      SIMULATION RESULTS SUMMARY TABLE\n');
fprintf('========================================================================================================\n');
fprintf('%-34s | %-11s | %-10s | %-12s | %-12s | %-12s | %-18s\n', ...
    'Controller Experiment', 'Initial SOC', 'Final SOC', 'Energy (kJ)', 'Energy (Wh)', 'Travel Time', 'Mission Status');
fprintf('--------------------------------------------------------------------------------------------------------\n');

csv_fid = fopen('results_summary.csv', 'w');
fprintf(csv_fid, 'Controller,Initial_SOC_pct,Final_SOC_pct,Energy_Used_kJ,Energy_Used_Wh,Travel_Time_s,Mission_Status\n');

for k = 1:num_exp
    fprintf('%-34s | %10.1f%% | %9.1f%% | %11.2f  | %11.2f  | %10.1f s | %-18s\n', ...
        results(k).name, results(k).initial_soc, results(k).final_soc, ...
        results(k).energy_used_kJ, results(k).energy_used_Wh, results(k).travel_time, results(k).mission_status);
    
    fprintf(csv_fid, '"%s",%.1f,%.1f,%.2f,%.2f,%.1f,"%s"\n', ...
        results(k).name, results(k).initial_soc, results(k).final_soc, ...
        results(k).energy_used_kJ, results(k).energy_used_Wh, results(k).travel_time, results(k).mission_status);
end
fclose(csv_fid);
fprintf('========================================================================================================\n');
fprintf('Summary table saved to results_summary.csv\n\n');

%% Step 6: Generate the 6 Required Comparative Graphs
fprintf('[4/5] Generating and exporting 6 required comparative graphs...\n');

% Set clean plotting styling with white background
set(groot, 'DefaultFigureColor', 'w');
set(groot, 'DefaultAxesColor', 'w');
set(groot, 'DefaultAxesXColor', [0.15 0.15 0.15]);
set(groot, 'DefaultAxesYColor', [0.15 0.15 0.15]);
set(groot, 'DefaultTextColor', [0.1 0.1 0.1]);
set(groot, 'DefaultLegendColor', 'w');
set(groot, 'DefaultLegendTextColor', [0.1 0.1 0.1]);
set(groot, 'DefaultLegendEdgeColor', [0.8 0.8 0.8]);
set(groot, 'DefaultAxesFontSize', 11);
set(groot, 'DefaultLineLineWidth', 1.8);
set(groot, 'DefaultAxesGridAlpha', 0.5);
set(groot, 'DefaultAxesGridColor', [0.85 0.85 0.85]);

%% Graph 1: Desired Speed vs Actual Speed (PID vs Battery-Aware MPC)
fig1 = figure('Name', 'Graph 1: Desired vs Actual Speed', 'Position', [100, 100, 950, 600], 'Color', 'w');
subplot(2, 1, 1);
plot(results(1).t, results(1).u_ref, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Target Speed (u_{ref})'); hold on;
plot(results(1).t, results(1).speed, 'Color', results(1).color, 'LineWidth', 2.0, 'DisplayName', 'Actual Speed (PID)');
grid on; xlim([0, 1000]); ylim([0, 2.5]);
ylabel('Speed [m/s]'); title('Experiment 1: PID Speed Tracking (Fixed Cruise @ 2 m/s)');
legend('Location', 'southeast');

subplot(2, 1, 2);
plot(results(3).t, results(3).u_ref, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Adaptive Target Speed (u_{ref})'); hold on;
plot(results(3).t, results(3).speed, 'Color', results(3).color, 'LineWidth', 2.0, 'DisplayName', 'Actual Speed (Battery-Aware MPC)');
grid on; xlim([0, 1000]); ylim([0, 2.5]);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Experiment 3: Battery-Aware MPC Speed Tracking & Adaptation');
legend('Location', 'southeast');
saveas(fig1, 'graph1_desired_vs_actual_speed.png');

%% Graph 2: PID vs MPC Speed (Direct Comparison)
fig2 = figure('Name', 'Graph 2: PID vs MPC Speed Comparison', 'Position', [150, 150, 900, 520], 'Color', 'w');
plot(results(1).t, results(1).speed, 'Color', results(1).color, 'LineWidth', 2.0, 'DisplayName', 'PID (80% Battery)'); hold on;
plot(results(2).t, results(2).speed, 'Color', results(2).color, 'LineWidth', 2.0, 'DisplayName', 'Standard MPC (80% Battery)');
plot(results(3).t, results(3).speed, 'Color', results(3).color, 'LineWidth', 2.0, 'DisplayName', 'Battery-Aware MPC (80% Battery)');
yline(2.0, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Nominal Cruise (2.0 m/s)');
grid on; xlim([0, 650]); ylim([0, 2.5]);
xlabel('Time [s]'); ylabel('USV Speed [m/s]');
title('PID vs MPC Speed Response & Energy-Aware Dynamic Regulation');
legend('Location', 'northeast');
saveas(fig2, 'graph2_pid_vs_mpc_speed.png');

%% Graph 3: Battery SOC vs Time (All 5 Experiments)
fig3 = figure('Name', 'Graph 3: Battery SOC vs Time', 'Position', [200, 200, 920, 540], 'Color', 'w');
for k = 1:num_exp
    plot(results(k).t, results(k).soc, 'Color', results(k).color, 'LineWidth', 2.2, 'DisplayName', results(k).name); hold on;
end
yline(60, 'Color', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1.2, 'DisplayName', 'Threshold High (60%)');
yline(30, 'Color', [0.6 0.2 0.2], 'LineStyle', '--', 'LineWidth', 1.2, 'DisplayName', 'Threshold Low (30%)');
grid on; xlim([0, 1050]); ylim([0, 85]);
xlabel('Time [s]'); ylabel('Battery SOC [%]');
title('Battery State-of-Charge (SOC) Depletion Profiles across 5 Experiments');
legend('Location', 'southwest');
saveas(fig3, 'graph3_battery_soc_vs_time.png');

%% Graph 4: Power Consumption vs Time
fig4 = figure('Name', 'Graph 4: Power Consumption vs Time', 'Position', [250, 250, 920, 520], 'Color', 'w');
for k = 1:num_exp
    plot(results(k).t, results(k).power, 'Color', results(k).color, 'LineWidth', 1.8, 'DisplayName', results(k).name); hold on;
end
grid on; xlim([0, 750]); ylim([0, 320]);
xlabel('Time [s]'); ylabel('Electrical / Propulsive Power [W]');
title('USV Power Consumption Profile (P = T \cdot u + P_{aux})');
legend('Location', 'northeast');
saveas(fig4, 'graph4_power_consumption_vs_time.png');

%% Graph 5: Distance Remaining vs Time
fig5 = figure('Name', 'Graph 5: Distance vs Time', 'Position', [300, 300, 920, 520], 'Color', 'w');
for k = 1:num_exp
    plot(results(k).t, results(k).dist_rem, 'Color', results(k).color, 'LineWidth', 2.0, 'DisplayName', results(k).name); hold on;
end
yline(0, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Destination Reached (0 m)');
grid on; xlim([0, 1050]); ylim([-10, 1050]);
xlabel('Time [s]'); ylabel('Distance Remaining [m]');
title('Distance Remaining to Destination (1000 m Straight-line Mission)');
legend('Location', 'northeast');
saveas(fig5, 'graph5_distance_vs_time.png');

%% Graph 6: Boat Position vs Time
fig6 = figure('Name', 'Graph 6: Boat Position vs Time', 'Position', [350, 350, 920, 520], 'Color', 'w');
for k = 1:num_exp
    plot(results(k).t, results(k).pos, 'Color', results(k).color, 'LineWidth', 2.2, 'DisplayName', results(k).name); hold on;
end
yline(params.mission.x_dest, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target Destination (1000 m)');
grid on; xlim([0, 1050]); ylim([0, 1050]);
xlabel('Time [s]'); ylabel('Boat Longitudinal Position [m]');
title('USV Trajectory Position Progression toward 1000 m Destination');
legend('Location', 'southeast');
saveas(fig6, 'graph6_boat_position_vs_time.png');

%% Step 7: Render Interactive Dashboard Figure
fprintf('[5/5] Generating USV Telemetry Dashboard...\n');
fig_dash = figure('Name', 'USV Battery-Aware MPC Mission Dashboard', 'Position', [120, 80, 1100, 720], 'Color', [0.95, 0.96, 0.98]);

% Top Banner
annotation('textbox', [0.03, 0.92, 0.94, 0.06], ...
    'String', 'USV BATTERY-AWARE MODEL PREDICTIVE CONTROL - FIRST-EVALUATION DASHBOARD', ...
    'FontSize', 14, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', ...
    'Color', [0.1, 0.15, 0.3], 'BackgroundColor', 'w', 'EdgeColor', [0.8, 0.82, 0.88], 'LineWidth', 1.2);

% Extract sample instant from Experiment 3 at t = 350 s
t_eval = 350.0;
idx_eval = find(results(3).t >= t_eval, 1);
d_soc = results(3).soc(idx_eval);
d_speed = results(3).speed(idx_eval);
d_dist = results(3).dist_rem(idx_eval);
d_range = results(3).est_range(idx_eval);
d_status = 'FEASIBLE';

% Telemetry Cards (Row 1)
card_x = [0.03, 0.19, 0.35, 0.51, 0.67, 0.83];
card_w = 0.14;
card_h = 0.12;
card_y = 0.77;

cards_data = {
    'Active Controller', 'Battery-Aware MPC', [0.1, 0.3, 0.6];
    'Battery SOC', sprintf('%.1f%%', d_soc), [0.12, 0.65, 0.28];
    'Current Speed', sprintf('%.2f m/s', d_speed), [0.1, 0.45, 0.8];
    'Distance Rem.', sprintf('%.0f m', d_dist), [0.85, 0.45, 0.1];
    'Estimated Range', sprintf('%.0f m', d_range), [0.2, 0.6, 0.3];
    'Mission Status', d_status, [0.12, 0.65, 0.28]
};

for i = 1:6
    annotation('textbox', [card_x(i), card_y, card_w, card_h], ...
        'BackgroundColor', 'w', ...
        'Color', cards_data{i, 3}, ...
        'FontSize', 10, 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'center', ...
        'VerticalAlignment', 'middle', ...
        'EdgeColor', [0.75, 0.78, 0.85], 'LineWidth', 1.2, ...
        'String', sprintf('%s\n\n%s', cards_data{i, 1}, cards_data{i, 2}));
end

% Telemetry Graphs on Dashboard
ax1 = subplot('Position', [0.06, 0.42, 0.42, 0.28]);
plot(results(3).t, results(3).speed, 'Color', [0.0, 0.45, 0.74], 'LineWidth', 2.0); hold on;
plot(results(3).t, results(3).u_ref, 'k--', 'LineWidth', 1.5);
grid on; xlim([0, 1000]); ylim([0, 2.5]);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Speed Response & Dynamic Adjustment');
legend('Speed u', 'Target u_{ref}', 'Location', 'northeast');

ax2 = subplot('Position', [0.55, 0.42, 0.40, 0.28]);
plot(results(3).t, results(3).soc, 'Color', [0.12, 0.65, 0.28], 'LineWidth', 2.2); hold on;
yline(60, 'k--', 'Threshold High (60%)');
yline(30, 'r--', 'Threshold Low (30%)');
grid on; xlim([0, 1000]); ylim([0, 85]);
xlabel('Time [s]'); ylabel('SOC [%]');
title('Battery State of Charge');

ax3 = subplot('Position', [0.06, 0.08, 0.42, 0.26]);
plot(results(3).t, results(3).dist_rem, 'Color', [0.85, 0.33, 0.10], 'LineWidth', 2.0); hold on;
plot(results(3).t, results(3).est_range, 'Color', [0.12, 0.65, 0.28], 'LineWidth', 2.0);
grid on; xlim([0, 1000]); ylim([0, 1200]);
xlabel('Time [s]'); ylabel('Distance [m]');
title('Distance Remaining vs Estimated Range');
legend('Distance Remaining', 'Estimated Range', 'Location', 'northeast');

ax4 = subplot('Position', [0.55, 0.08, 0.40, 0.26]);
plot(results(1).t, results(1).power, 'Color', results(1).color, 'LineWidth', 1.8); hold on;
plot(results(2).t, results(2).power, 'Color', results(2).color, 'LineWidth', 1.8);
plot(results(3).t, results(3).power, 'Color', results(3).color, 'LineWidth', 1.8);
grid on; xlim([0, 600]); ylim([0, 300]);
xlabel('Time [s]'); ylabel('Power [W]');
title('Power Expenditure Comparison');
legend('PID', 'Std MPC', 'Battery-Aware MPC', 'Location', 'northeast');

saveas(fig_dash, 'dashboard.png');

%% Save all simulation workspace variables
save('simulation_results.mat', 'params', 'experiments', 'results');
fprintf('All results and workspace saved to simulation_results.mat\n');
fprintf('Generated graph files:\n');
fprintf('  - graph1_desired_vs_actual_speed.png\n');
fprintf('  - graph2_pid_vs_mpc_speed.png\n');
fprintf('  - graph3_battery_soc_vs_time.png\n');
fprintf('  - graph4_power_consumption_vs_time.png\n');
fprintf('  - graph5_distance_vs_time.png\n');
fprintf('  - graph6_boat_position_vs_time.png\n');
fprintf('  - dashboard.png\n');
fprintf('\nExecution completed successfully!\n');

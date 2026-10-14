%% REGENERATE_ALL_WHITE_OUTPUTS
% Generates all 24 project output images with crisp white backgrounds and maximum visibility.

clear; clc; close all;
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(root_dir));

% 1. Global Figure and Axes Defaults for Pure White Theme
set(0, 'DefaultFigureColor', 'w');
set(0, 'DefaultAxesColor', 'w');
set(0, 'DefaultAxesXColor', [0.15 0.15 0.15]);
set(0, 'DefaultAxesYColor', [0.15 0.15 0.15]);
set(0, 'DefaultAxesZColor', [0.15 0.15 0.15]);
set(0, 'DefaultTextColor', [0.10 0.10 0.10]);
set(0, 'DefaultAxesGridColor', [0.85 0.85 0.85]);
set(0, 'DefaultAxesGridAlpha', 0.6);
set(0, 'DefaultLegendColor', 'w');
set(0, 'DefaultLegendTextColor', [0.10 0.10 0.10]);
set(0, 'DefaultLegendEdgeColor', [0.80 0.80 0.80]);
set(0, 'DefaultAxesFontSize', 11);
set(0, 'DefaultAxesFontName', 'Arial');

fprintf('========================================================================\n');
fprintf('  REGENERATING ALL PROJECT OUTPUTS WITH HIGH-VISIBILITY WHITE BACKGROUNDS\n');
fprintf('========================================================================\n\n');

%% 2. Generate Figures 1 through 15
fprintf('[1/4] Generating 15 publication benchmark figures (fig1 to fig15)...\n');
load(fullfile(root_dir, 'results', 'all_simulation_results.mat'), 'all_results');
generate_all_figures(all_results, root_dir);

%% 3. Generate Telemetry Dashboard
fprintf('[2/4] Generating telemetry dashboard (dashboard.png)...\n');
create_dashboard(all_results{3}, root_dir);

%% 4. Generate 6 Comparative Graphs (graph1 to graph6)
fprintf('[3/4] Generating 6 comparative graphs (graph1 to graph6)...\n');
res1 = all_results{1}; % PID (80%)
res2 = all_results{2}; % Conv MPC (80%)
res3 = all_results{3}; % Bat-Aware MPC (80%)
res4 = all_results{4}; % Bat-Aware MPC (60%)
res5 = all_results{5}; % Bat-Aware MPC (40%)

c_pid   = [0.85 0.30 0.15]; % Orange-Red
c_conv  = [0.10 0.45 0.85]; % Blue
c_ba80  = [0.05 0.65 0.25]; % Deep Green
c_ba60  = [0.70 0.20 0.70]; % Purple
c_ba40  = [0.85 0.55 0.05]; % Amber

% --- Graph 1: Desired Speed vs Actual Speed ---
fig_g1 = figure('Name', 'Graph 1: Desired vs Actual Speed', 'Position', [100, 100, 950, 600], 'Color', 'w', 'Visible', 'off');
subplot(2, 1, 1);
plot(res1.t, res1.u_ref, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Target Speed (u_{ref})'); hold on;
plot(res1.t, res1.u, 'Color', c_pid, 'LineWidth', 2.0, 'DisplayName', 'Actual Speed (PID)');
grid on; xlim([0, 1000]); ylim([0, 2.5]);
ylabel('Speed [m/s]'); title('Experiment 1: PID Speed Tracking (Fixed Cruise @ 2 m/s)', 'FontWeight', 'bold');
legend('Location', 'southeast', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);

subplot(2, 1, 2);
plot(res3.t, res3.u_ref, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Adaptive Target Speed (u_{ref})'); hold on;
plot(res3.t, res3.u, 'Color', c_ba80, 'LineWidth', 2.0, 'DisplayName', 'Actual Speed (Battery-Aware MPC)');
grid on; xlim([0, 1000]); ylim([0, 2.5]);
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Experiment 3: Battery-Aware MPC Speed Tracking & Adaptation', 'FontWeight', 'bold');
legend('Location', 'southeast', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
saveas(fig_g1, fullfile(root_dir, 'graph1_desired_vs_actual_speed.png'));

% --- Graph 2: PID vs MPC Speed Comparison ---
fig_g2 = figure('Name', 'Graph 2: PID vs MPC Speed Comparison', 'Position', [150, 150, 900, 520], 'Color', 'w', 'Visible', 'off');
plot(res1.t, res1.u, 'Color', c_pid,  'LineWidth', 2.0, 'DisplayName', 'PID (80% Battery)'); hold on;
plot(res2.t, res2.u, 'Color', c_conv, 'LineWidth', 2.0, 'DisplayName', 'Standard MPC (80% Battery)');
plot(res3.t, res3.u, 'Color', c_ba80, 'LineWidth', 2.0, 'DisplayName', 'Battery-Aware MPC (80% Battery)');
yline(2.0, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Nominal Cruise (2.0 m/s)');
grid on; xlim([0, 650]); ylim([0, 2.5]);
xlabel('Time [s]'); ylabel('USV Speed [m/s]');
title('PID vs MPC Speed Response & Energy-Aware Dynamic Regulation', 'FontWeight', 'bold');
legend('Location', 'northeast', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
saveas(fig_g2, fullfile(root_dir, 'graph2_pid_vs_mpc_speed.png'));

% --- Graph 3: Battery SOC vs Time ---
fig_g3 = figure('Name', 'Graph 3: Battery SOC vs Time', 'Position', [200, 200, 920, 540], 'Color', 'w', 'Visible', 'off');
plot(res1.t, res1.SOC, 'Color', c_pid,  'LineWidth', 2.0, 'DisplayName', 'Exp 1: PID (80% SOC)'); hold on;
plot(res2.t, res2.SOC, 'Color', c_conv, 'LineWidth', 2.0, 'DisplayName', 'Exp 2: Standard MPC (80% SOC)');
plot(res3.t, res3.SOC, 'Color', c_ba80, 'LineWidth', 2.2, 'DisplayName', 'Exp 3: Bat-Aware MPC (80% SOC)');
plot(res4.t, res4.SOC, 'Color', c_ba60, 'LineWidth', 2.0, 'DisplayName', 'Exp 4: Bat-Aware MPC (60% SOC)');
plot(res5.t, res5.SOC, 'Color', c_ba40, 'LineWidth', 2.0, 'DisplayName', 'Exp 5: Bat-Aware MPC (40% SOC)');
yline(60, 'Color', [0.3 0.3 0.3], 'LineStyle', '--', 'LineWidth', 1.2, 'DisplayName', 'Threshold High (60%)');
yline(30, 'Color', [0.7 0.2 0.2], 'LineStyle', '--', 'LineWidth', 1.2, 'DisplayName', 'Threshold Low (30%)');
grid on; xlim([0, 1050]); ylim([0, 85]);
xlabel('Time [s]'); ylabel('Battery SOC [%]');
title('Battery State-of-Charge (SOC) Depletion Profiles across 5 Experiments', 'FontWeight', 'bold');
legend('Location', 'southwest', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
saveas(fig_g3, fullfile(root_dir, 'graph3_battery_soc_vs_time.png'));

% --- Graph 4: Power Consumption vs Time ---
fig_g4 = figure('Name', 'Graph 4: Power Consumption vs Time', 'Position', [250, 250, 920, 520], 'Color', 'w', 'Visible', 'off');
plot(res1.t, res1.P, 'Color', c_pid,  'LineWidth', 1.6, 'DisplayName', 'Exp 1: PID (80% SOC)'); hold on;
plot(res2.t, res2.P, 'Color', c_conv, 'LineWidth', 1.6, 'DisplayName', 'Exp 2: Standard MPC (80% SOC)');
plot(res3.t, res3.P, 'Color', c_ba80, 'LineWidth', 1.8, 'DisplayName', 'Exp 3: Bat-Aware MPC (80% SOC)');
grid on; xlim([0, 750]);
xlabel('Time [s]'); ylabel('Electrical / Propulsive Power [W]');
title('USV Power Consumption Profile (P = T \cdot u + P_{aux})', 'FontWeight', 'bold');
legend('Location', 'northeast', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
saveas(fig_g4, fullfile(root_dir, 'graph4_power_consumption_vs_time.png'));

% --- Graph 5: Distance Remaining vs Time ---
fig_g5 = figure('Name', 'Graph 5: Distance vs Time', 'Position', [300, 300, 920, 520], 'Color', 'w', 'Visible', 'off');
plot(res1.t, res1.d_dest, 'Color', c_pid,  'LineWidth', 2.0, 'DisplayName', 'Exp 1: PID (80% SOC)'); hold on;
plot(res2.t, res2.d_dest, 'Color', c_conv, 'LineWidth', 2.0, 'DisplayName', 'Exp 2: Standard MPC (80% SOC)');
plot(res3.t, res3.d_dest, 'Color', c_ba80, 'LineWidth', 2.2, 'DisplayName', 'Exp 3: Bat-Aware MPC (80% SOC)');
yline(0, 'k-', 'LineWidth', 1.2, 'DisplayName', 'Destination Reached (0 m)');
grid on; xlim([0, 1050]);
xlabel('Time [s]'); ylabel('Distance Remaining [m]');
title('Distance Remaining to Destination', 'FontWeight', 'bold');
legend('Location', 'northeast', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
saveas(fig_g5, fullfile(root_dir, 'graph5_distance_vs_time.png'));

% --- Graph 6: Boat Position vs Time ---
fig_g6 = figure('Name', 'Graph 6: Boat Position vs Time', 'Position', [350, 350, 920, 520], 'Color', 'w', 'Visible', 'off');
plot(res1.t, res1.x, 'Color', c_pid,  'LineWidth', 2.0, 'DisplayName', 'Exp 1: PID (80% SOC)'); hold on;
plot(res2.t, res2.x, 'Color', c_conv, 'LineWidth', 2.0, 'DisplayName', 'Exp 2: Standard MPC (80% SOC)');
plot(res3.t, res3.x, 'Color', c_ba80, 'LineWidth', 2.2, 'DisplayName', 'Exp 3: Bat-Aware MPC (80% SOC)');
grid on; xlim([0, 1050]);
xlabel('Time [s]'); ylabel('Boat Longitudinal Position [m]');
title('USV Longitudinal Position Progression toward Destination', 'FontWeight', 'bold');
legend('Location', 'southeast', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15]);
saveas(fig_g6, fullfile(root_dir, 'graph6_boat_position_vs_time.png'));

%% 5. Generate Boat Navigation & Trajectory Figures
fprintf('[4/4] Generating Boat Navigation with Obstacles figures...\n');
load(fullfile(root_dir, 'Boat_Project_Results.mat'), 'grid', 'start', 'goal', 'path', 'X', 'Y');

fig_boat = figure('Name', 'Boat Navigation with Obstacles', 'Color', 'w', 'Position', [100, 100, 1000, 750], 'Visible', 'off');
ax_b = axes('Color', 'w');
imagesc(grid);
colormap(ax_b, [1 1 1; 0.35 0.40 0.48]); % 0=white free water, 1=slate grey obstacles
axis equal tight;
axis ij;
hold on; grid on;
set(ax_b, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], ...
    'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.6, 'FontSize', 11);

h_plan  = plot(path(:,2), path(:,1), 'b--', 'LineWidth', 2.5, 'DisplayName', 'Theta* Planned Path');
plot(path(:,2), path(:,1), 'bo', 'MarkerFaceColor', 'b', 'MarkerSize', 6, 'HandleVisibility', 'off');
h_act   = plot(X/10.0, Y, 'r-', 'LineWidth', 2.2, 'DisplayName', 'Actual Boat Trajectory');
h_start = plot(start(2), start(1), 'go', 'MarkerFaceColor', [0.1 0.7 0.2], 'MarkerSize', 11, 'DisplayName', 'Start Point');
h_goal  = plot(goal(2), goal(1), 'r*', 'MarkerSize', 14, 'LineWidth', 2.2, 'DisplayName', 'Destination');

title('Boat Navigation with Obstacles (Theta* Any-Angle Path & USV Trajectory)', ...
    'FontSize', 13, 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
xlabel('X Position (Grid Cells)', 'FontSize', 11, 'Color', [0.15 0.15 0.15]);
ylabel('Y Position (Grid Cells)', 'FontSize', 11, 'Color', [0.15 0.15 0.15]);
legend([h_plan, h_act, h_start, h_goal], 'Location', 'northeast', ...
    'Color', 'w', 'EdgeColor', [0.8 0.8 0.8], 'FontSize', 10);

saveas(fig_boat, fullfile(root_dir, 'Boat_Navigation_with_Obstacles.png'));
saveas(fig_boat, fullfile(root_dir, 'Boat_Trajectory.png'));

fprintf('\n========================================================================\n');
fprintf('  ALL 24 OUTPUT IMAGES SUCCESSFULLY REGENERATED WITH WHITE BACKGROUNDS!  \n');
fprintf('========================================================================\n');

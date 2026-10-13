%% =========================================================================
%  COMPLETE PROJECT DEMONSTRATION SCRIPT
%  Battery-Aware MPC for Electric Tourist Boats
%  
%  Run this script in MATLAB to demonstrate ALL project components
%  Press any key to advance between sections
% =========================================================================

clc; clear; close all;

% Add all project folders to path
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(root_dir));

fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║  BATTERY-AWARE MODEL PREDICTIVE SPEED CONTROL                  ║\n');
fprintf('║  FOR ELECTRIC TOURIST BOATS - COMPLETE DEMONSTRATION           ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');

%% ========================================================================
%  DEMO 1: ENVIRONMENT & MAP (River/Waterway with Obstacles)
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 1: ENVIRONMENT - Simulated Tourist Boat Waterway\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → Winding river/waterway (1000m x 600m)\n');
fprintf('  → Island obstacle in waterway\n');
fprintf('  → 4 floating obstacles / restricted zones\n');
fprintf('  → Start point and Destination defined\n\n');

% Run the environment script inline
mapLength = 1000; mapWidth = 600; resolution = 2;
x_env = 0:resolution:mapLength; y_env = 0:resolution:mapWidth;
[X_env, Y_env] = meshgrid(x_env, y_env);
centerY = 300 + 65*sin(2*pi*X_env/mapLength) + 25*sin(4*pi*X_env/mapLength);
channelWidth = 190 + 20*sin(2*pi*X_env/mapLength);
occupancyMap = abs(Y_env - centerY) > channelWidth/2;
island = ((X_env - 510)/55).^2 + ((Y_env - 330)/38).^2 <= 1;
occupancyMap(island) = 1;
obstacles_env = [250 260 15; 370 365 18; 680 250 20; 800 350 16];
for k = 1:size(obstacles_env, 1)
    obsMask = (X_env - obstacles_env(k,1)).^2 + (Y_env - obstacles_env(k,2)).^2 <= obstacles_env(k,3)^2;
    occupancyMap(obsMask) = 1;
end
startPoint = [40, 300]; goalPoint = [950, 300];

figure('Name', 'DEMO 1: Environment', 'Color', 'w', 'Position', [50 50 900 500]);
imagesc(x_env, y_env, occupancyMap);
set(gca, 'YDir', 'normal'); axis equal tight;
colormap([0.2 0.65 0.85; 0.35 0.35 0.35]);
hold on;
plot(startPoint(1), startPoint(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 14);
plot(goalPoint(1), goalPoint(2), 'r*', 'MarkerSize', 16, 'LineWidth', 2);
title('DEMO 1: Simulated Tourist Boat Waterway Environment', 'FontSize', 14);
xlabel('X Position (m)'); ylabel('Y Position (m)');
legend('Start (Dock)', 'Destination', 'Location', 'best');
grid on;

save('boat_environment_data.mat', 'occupancyMap', 'x_env', 'y_env', 'resolution', 'startPoint', 'goalPoint');
fprintf('  ✓ Environment created. Press any key for next demo...\n');
pause;

%% ========================================================================
%  DEMO 2: THETA* PATH PLANNING
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 2: THETA* PATH PLANNING (Any-Angle Optimal Path)\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → Theta* algorithm: any-angle path (not grid-locked)\n');
fprintf('  → Line-of-Sight checks for obstacle avoidance\n');
fprintf('  → 8-connected grid search with parent rewiring\n');
fprintf('  → Produces smooth waypoints for MPC to follow\n\n');

% Run Theta* on a demo grid
grid_demo = zeros(100, 100);
grid_demo(25:65, 35:42) = 1;
grid_demo(45:85, 65:72) = 1;
grid_demo(10:20, 75:90) = 1;
start_demo = [5, 5]; goal_demo = [90, 90];

path_demo = theta_star(grid_demo, start_demo, goal_demo);

figure('Name', 'DEMO 2: Theta* Path Planning', 'Color', 'w', 'Position', [100 100 800 700]);
subplot(1,2,1);
imagesc(grid_demo); colormap(flipud(gray)); axis equal; axis ij; hold on; grid on;
if ~isempty(path_demo)
    plot(path_demo(:,2), path_demo(:,1), 'b-', 'LineWidth', 3);
    plot(start_demo(2), start_demo(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 12);
    plot(goal_demo(2), goal_demo(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 12);
end
title('Theta* Path on Grid (Any-Angle)', 'FontSize', 12);
legend('Theta* Path', 'Start', 'Goal');
xlabel('Column'); ylabel('Row');

% Show waypoint count
subplot(1,2,2);
if ~isempty(path_demo)
    segLens = sqrt(sum(diff(path_demo).^2, 2));
    bar(segLens, 'FaceColor', [0.2 0.6 0.9]);
    title(sprintf('Theta* Segment Lengths (%d waypoints)', size(path_demo, 1)), 'FontSize', 12);
    xlabel('Segment #'); ylabel('Length (cells)');
    grid on;
    fprintf('  Path found: %d waypoints, total length: %.2f cells\n', size(path_demo, 1), sum(segLens));
end

fprintf('  ✓ Theta* demo complete. Press any key for next demo...\n');
pause;

%% ========================================================================
%  DEMO 3: 3-DOF MATHEMATICAL MODEL
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 3: 3-DOF MATHEMATICAL MODEL (Surge, Sway, Yaw)\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → States: η = [x, y, ψ] (position + heading)\n');
fprintf('  →          ν = [u, v, r] (surge, sway, yaw rate)\n');
fprintf('  → Rigid-body mass matrix M_RB (180 kg vessel)\n');
fprintf('  → Added mass matrix M_A (hydrodynamic)\n');
fprintf('  → Coriolis & centripetal matrix C(ν)\n');
fprintf('  → Nonlinear damping D(ν_r) (linear + quadratic drag)\n');
fprintf('  → Equation: M·ν̇ = τ_ctrl + τ_env − C·ν − D·ν_r\n');
fprintf('  → Integration: RK4 (4th-order Runge-Kutta)\n\n');

p_usv = USV_parameters();

% Show the matrices at a sample velocity
nu_sample = [1.5; 0.2; 0.1];  % u=1.5 m/s, v=0.2 m/s, r=0.1 rad/s
nu_r_sample = nu_sample;
[M, C, D, tau_D] = hydrodynamics(nu_sample, nu_r_sample, p_usv);

figure('Name', 'DEMO 3: 3-DOF Model Matrices', 'Color', 'w', 'Position', [150 150 900 600]);

subplot(2,2,1);
imagesc(M); colorbar; axis equal tight;
title(sprintf('M (Inertia) = M_{RB} + M_A\nm = %.0f kg, Iz = %.0f kg·m²', p_usv.m, p_usv.Iz), 'FontSize', 11);
set(gca, 'XTick', 1:3, 'YTick', 1:3, 'XTickLabel', {'u','v','r'}, 'YTickLabel', {'u','v','r'});

subplot(2,2,2);
imagesc(C); colorbar; axis equal tight;
title('C(ν) Coriolis & Centripetal', 'FontSize', 11);
set(gca, 'XTick', 1:3, 'YTick', 1:3, 'XTickLabel', {'u','v','r'}, 'YTickLabel', {'u','v','r'});

subplot(2,2,3);
imagesc(D); colorbar; axis equal tight;
title('D(ν_r) Hydrodynamic Damping', 'FontSize', 11);
set(gca, 'XTick', 1:3, 'YTick', 1:3, 'XTickLabel', {'u','v','r'}, 'YTickLabel', {'u','v','r'});

subplot(2,2,4);
bar(tau_D, 'FaceColor', [0.85 0.33 0.1]);
set(gca, 'XTickLabel', {'Surge τ_D(1)', 'Sway τ_D(2)', 'Yaw τ_D(3)'});
title('Damping Force Vector τ_D = D·ν_r', 'FontSize', 11);
ylabel('Force [N] / Moment [N·m]'); grid on;

sgtitle('DEMO 3: 3-DOF Hydrodynamic Matrices (Fossen 2011)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('  M matrix (3x3):\n');
disp(M);
fprintf('  Damping forces τ_D at u=1.5, v=0.2, r=0.1:\n');
fprintf('    Surge drag: %.2f N\n', tau_D(1));
fprintf('    Sway drag:  %.2f N\n', tau_D(2));
fprintf('    Yaw drag:   %.2f N·m\n', tau_D(3));

fprintf('\n  ✓ 3-DOF model shown. Press any key for next demo...\n');
pause;

%% ========================================================================
%  DEMO 4: ENVIRONMENTAL DISTURBANCES
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 4: ENVIRONMENTAL DISTURBANCES (Current, Wind, Waves)\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → Water current: 0.35 m/s (relative flow model)\n');
fprintf('  → Wind: 4.5 m/s (aerodynamic drag: frontal + lateral)\n');
fprintf('  → Waves: 1st-order oscillatory + 2nd-order drift\n');
fprintf('  → All forces computed in body frame\n\n');

p_env = environment_parameters();
eta_demo = [100; 300; deg2rad(45)];
nu_demo = [1.5; 0.1; 0.05];

t_vec = 0:0.1:60;
tau_wind_log = zeros(3, length(t_vec));
tau_wave_log = zeros(3, length(t_vec));
tau_total_log = zeros(3, length(t_vec));

for i = 1:length(t_vec)
    [tau_e, ~, ~] = environmental_forces(eta_demo, nu_demo, t_vec(i), p_env, p_usv);
    tau_total_log(:, i) = tau_e;
end

figure('Name', 'DEMO 4: Environmental Disturbances', 'Color', 'w', 'Position', [200 100 900 600]);

subplot(3,1,1);
plot(t_vec, tau_total_log(1,:), 'r-', 'LineWidth', 1.8);
ylabel('Surge [N]'); title('DEMO 4: Environmental Force Disturbances Over Time', 'FontSize', 13);
grid on; legend('τ_{env,surge} (Wind + Waves)');

subplot(3,1,2);
plot(t_vec, tau_total_log(2,:), 'b-', 'LineWidth', 1.8);
ylabel('Sway [N]'); grid on; legend('τ_{env,sway}');

subplot(3,1,3);
plot(t_vec, tau_total_log(3,:), 'm-', 'LineWidth', 1.8);
xlabel('Time [s]'); ylabel('Yaw [N·m]'); grid on; legend('τ_{env,yaw}');

fprintf('  ✓ Environmental forces plotted. Press any key for next demo...\n');
pause;

%% ========================================================================
%  DEMO 5: BATTERY SOC MODEL
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 5: BATTERY STATE OF CHARGE (SOC) MODEL\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → 1200 Wh LiFePO4 battery pack (24V, 50Ah)\n');
fprintf('  → Coulomb counting: dSOC = -(P·dt / E_cap) × 100\n');
fprintf('  → Equivalent circuit: V_oc(SOC), R_int, I_batt\n');
fprintf('  → Hotel load: 35W continuous\n');
fprintf('  → Reserve threshold: 20%% SOC\n\n');

p_bat = battery_parameters();
dt_bat = 1.0;
SOC_trace = zeros(1, 1200);
V_trace = zeros(1, 1200);
I_trace = zeros(1, 1200);
P_trace = zeros(1, 1200);
SOC_cur = 80.0; E_cur = 0;

for i = 1:1200
    P_prop_demo = 300 + 50*sin(0.01*i);  % Varying propulsion load
    [SOC_cur, E_cur, P_b, V_t, I_b] = battery_model(P_prop_demo, false, SOC_cur, E_cur, dt_bat, p_bat);
    SOC_trace(i) = SOC_cur;
    V_trace(i) = V_t;
    I_trace(i) = I_b;
    P_trace(i) = P_b;
end

figure('Name', 'DEMO 5: Battery SOC Model', 'Color', 'w', 'Position', [250 80 900 700]);

subplot(2,2,1);
plot((1:1200)/60, SOC_trace, 'g-', 'LineWidth', 2.5);
yline(20, 'r--', 'Reserve (20%)', 'LineWidth', 1.5);
xlabel('Time [min]'); ylabel('SOC [%]');
title('Battery State of Charge Depletion'); grid on; ylim([0 100]);

subplot(2,2,2);
plot((1:1200)/60, V_trace, 'b-', 'LineWidth', 2);
xlabel('Time [min]'); ylabel('Terminal Voltage [V]');
title('Battery Terminal Voltage'); grid on;

subplot(2,2,3);
plot((1:1200)/60, I_trace, 'r-', 'LineWidth', 2);
xlabel('Time [min]'); ylabel('Current [A]');
title('Battery Discharge Current'); grid on;

subplot(2,2,4);
plot((1:1200)/60, P_trace, 'm-', 'LineWidth', 2);
xlabel('Time [min]'); ylabel('Power [W]');
title('Total Battery Power (P_{prop} + P_{hotel})'); grid on;

sgtitle('DEMO 5: Battery SOC & Equivalent Circuit Model', 'FontSize', 14, 'FontWeight', 'bold');
fprintf('  Capacity: %d Wh | Hotel Load: %d W | Reserve: %d%%\n', p_bat.capacity_Wh, p_bat.P_hotel, p_bat.soc_reserve_pct);
fprintf('  ✓ Battery model shown. Press any key for next demo...\n');
pause;

%% ========================================================================
%  DEMO 6: MPC CONTROLLER DESIGN (Receding Horizon)
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 6: MPC CONTROLLER DESIGN (Receding Horizon)\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → Prediction Horizon Np = 20 steps (5.0 sec look-ahead)\n');
fprintf('  → Control Horizon    Nc = 6 steps  (1.5 sec)\n');
fprintf('  → Sample Time        Ts = 0.25 sec (4 Hz)\n');
fprintf('  → States: x = [u; r; ψ; e_y] (4 states)\n');
fprintf('  → Inputs: u = [T; N]          (2 inputs: thrust, yaw moment)\n');
fprintf('  → Linearized around operating point\n');
fprintf('  → QP solver: projected gradient with warm-start\n');
fprintf('  → Battery-Aware: adaptive power penalty w_p(SOC)\n');
fprintf('  → Only first control [T_cmd, N_cmd] applied each step\n\n');

p_mpc = MPC_parameters();
fprintf('  MPC Configuration:\n');
fprintf('    Np (prediction horizon) = %d steps (%.1f sec)\n', p_mpc.Np, p_mpc.Np * p_mpc.Ts);
fprintf('    Nc (control horizon)    = %d steps (%.1f sec)\n', p_mpc.Nc, p_mpc.Nc * p_mpc.Ts);
fprintf('    Ts (sample time)        = %.2f sec\n', p_mpc.Ts);
fprintf('    w_speed  = %.1f | w_cte   = %.1f | w_psi   = %.1f\n', p_mpc.weights.w_speed, p_mpc.weights.w_cte, p_mpc.weights.w_psi);
fprintf('    w_power  = %.2f (battery energy penalty)\n', p_mpc.weights.w_power);
fprintf('    T_max    = %.0f N | N_max   = %.0f N·m\n', p_mpc.bounds.T_max, p_mpc.bounds.N_max);
fprintf('    Solver: max %d iter, tol = %.0e\n', p_mpc.solver.max_iter, p_mpc.solver.tol);

% Show receding horizon concept diagram
figure('Name', 'DEMO 6: MPC Receding Horizon', 'Color', 'w', 'Position', [300 100 800 400]);
t_hor = 0:p_mpc.Ts:(p_mpc.Np * p_mpc.Ts);
u_ref_hor = 2.0 * ones(size(t_hor));
u_pred = 0.5 + 1.5 * (1 - exp(-t_hor / 2));
u_ctrl = u_pred(1:p_mpc.Nc+1);
t_ctrl = t_hor(1:p_mpc.Nc+1);

plot(t_hor, u_ref_hor, 'k--', 'LineWidth', 2, 'DisplayName', 'Reference u_{ref}'); hold on;
plot(t_hor, u_pred, 'b-o', 'LineWidth', 1.8, 'MarkerSize', 4, 'DisplayName', sprintf('Prediction Horizon (Np=%d)', p_mpc.Np));
plot(t_ctrl, u_ctrl, 'r-s', 'LineWidth', 2.5, 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', sprintf('Control Horizon (Nc=%d)', p_mpc.Nc));
plot(t_ctrl(1), u_ctrl(1), 'gp', 'MarkerSize', 20, 'MarkerFaceColor', 'g', 'DisplayName', 'Applied Control (1st step only)');
xline(t_ctrl(end), 'r:', 'Nc boundary', 'LineWidth', 1.2);
xline(t_hor(end), 'b:', 'Np boundary', 'LineWidth', 1.2);
xlabel('Time Horizon [s]'); ylabel('Surge Speed [m/s]');
title(sprintf('DEMO 6: MPC Receding Horizon Concept (Np=%d, Nc=%d, Ts=%.2fs)', p_mpc.Np, p_mpc.Nc, p_mpc.Ts), 'FontSize', 13);
legend('Location', 'southeast'); grid on;

fprintf('\n  ✓ MPC design shown. Press any key for FULL SIMULATION...\n');
pause;

%% ========================================================================
%  DEMO 7: FULL CLOSED-LOOP SIMULATION (15 Experiments)
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 7: RUNNING FULL SIMULATION (15 Experiments)\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  → Experiment 1-3:  PID vs Conv MPC vs Battery-Aware MPC\n');
fprintf('  → Experiment 4-6:  Different initial SOC (60%%, 40%%, 20%%)\n');
fprintf('  → Experiment 7-10: Environmental disturbances\n');
fprintf('  → Experiment 11:   Shortest vs energy route\n');
fprintf('  → Experiment 12-14: Emergency/dock/return scenarios\n');
fprintf('  → Experiment 15:   Obstacle avoidance\n');
fprintf('  → This may take 2-5 minutes...\n\n');

t_start = tic;
all_results = run_all_experiments();
t_elapsed = toc(t_start);
fprintf('\n  ✓ All 15 experiments completed in %.1f seconds!\n', t_elapsed);
fprintf('  Press any key to see RESULTS...\n');
pause;

%% ========================================================================
%  DEMO 8: RESULTS - TRAJECTORY & PATH TRACKING
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 8: RESULTS - Trajectory & Theta* Path Tracking\n');
fprintf('══════════════════════════════════════════════════════════════\n');

res1 = all_results{1}; res2 = all_results{2}; res3 = all_results{3};

figure('Name', 'DEMO 8: Trajectory Results', 'Color', 'w', 'Position', [50 50 1000 500]);
subplot(1,2,1);
% Draw obstacles
for k = 1:length(res3.map_data.obstacles)
    obs = res3.map_data.obstacles(k);
    if strcmp(obs.type, 'circle')
        theta_c = linspace(0, 2*pi, 100);
        fill(obs.center(1)+obs.radius*cos(theta_c), obs.center(2)+obs.radius*sin(theta_c), ...
            [0.8 0.6 0.4], 'FaceAlpha', 0.5, 'EdgeColor', [0.6 0.3 0.1], 'LineWidth', 1.5, 'HandleVisibility', 'off');
    elseif strcmp(obs.type, 'polygon')
        fill(obs.vertices(:,1), obs.vertices(:,2), [0.8 0.6 0.4], 'FaceAlpha', 0.5, 'HandleVisibility', 'off');
    end
    hold on;
end
plot(res1.x, res1.y, 'r--', 'LineWidth', 1.8, 'DisplayName', 'PID');
plot(res2.x, res2.y, 'b-.', 'LineWidth', 1.8, 'DisplayName', 'Conv MPC');
plot(res3.x, res3.y, 'g-',  'LineWidth', 2.5, 'DisplayName', 'Battery-Aware MPC');
plot(res3.waypoints(:,1), res3.waypoints(:,2), 'k*--', 'LineWidth', 1.5, 'MarkerSize', 8, 'DisplayName', 'Theta* Waypoints');
xlabel('X [m]'); ylabel('Y [m]');
title('2D Trajectory: 3 Controllers + Theta* Path'); legend('Location', 'best'); grid on; axis equal;

subplot(1,2,2);
plot(res3.x, res3.y, 'b-', 'LineWidth', 2.5, 'DisplayName', 'Actual Track'); hold on;
plot(res3.waypoints(:,1), res3.waypoints(:,2), 'ro--', 'LineWidth', 2, 'MarkerSize', 8, 'DisplayName', 'Theta* Planned Path');
xlabel('X [m]'); ylabel('Y [m]');
title('Theta* Path vs Actual Track'); legend('Location', 'best'); grid on; axis equal;

fprintf('  Theta* waypoints: %d\n', size(res3.waypoints, 1));
fprintf('  ✓ Press any key for speed & energy results...\n');
pause;

%% ========================================================================
%  DEMO 9: RESULTS - SPEED TRACKING & BATTERY PERFORMANCE
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 9: RESULTS - Speed Tracking & Battery Performance\n');
fprintf('══════════════════════════════════════════════════════════════\n');

figure('Name', 'DEMO 9: Speed & Battery', 'Color', 'w', 'Position', [80 50 1000 700]);

subplot(2,2,1);
plot(res1.t, res1.u, 'r--', 'LineWidth', 1.5, 'DisplayName', 'PID'); hold on;
plot(res2.t, res2.u, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Conv MPC');
plot(res3.t, res3.u, 'g-',  'LineWidth', 2, 'DisplayName', 'Bat-Aware MPC');
plot(res3.t, res3.u_ref, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Reference');
xlabel('Time [s]'); ylabel('Speed [m/s]');
title('Speed Tracking Comparison'); legend('Location', 'best'); grid on;

subplot(2,2,2);
plot(res1.t, res1.SOC, 'r--', 'LineWidth', 2, 'DisplayName', 'PID'); hold on;
plot(res2.t, res2.SOC, 'b-.', 'LineWidth', 2, 'DisplayName', 'Conv MPC');
plot(res3.t, res3.SOC, 'g-',  'LineWidth', 2.5, 'DisplayName', 'Bat-Aware MPC');
yline(20, 'k:', 'Reserve 20%', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('SOC [%]');
title('Battery SOC Depletion'); legend('Location', 'southwest'); grid on; ylim([0 100]);

subplot(2,2,3);
plot(res1.t, res1.P, 'r--', 'LineWidth', 1.5, 'DisplayName', 'PID'); hold on;
plot(res2.t, res2.P, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Conv MPC');
plot(res3.t, res3.P, 'g-',  'LineWidth', 2, 'DisplayName', 'Bat-Aware MPC');
xlabel('Time [s]'); ylabel('Power [W]');
title('Instantaneous Power Draw'); legend('Location', 'best'); grid on;

subplot(2,2,4);
d1 = [0; cumsum(sqrt(diff(res1.x).^2 + diff(res1.y).^2))];
d2 = [0; cumsum(sqrt(diff(res2.x).^2 + diff(res2.y).^2))];
d3 = [0; cumsum(sqrt(diff(res3.x).^2 + diff(res3.y).^2))];
plot(d1, res1.E_used_Wh, 'r--', 'LineWidth', 2, 'DisplayName', 'PID'); hold on;
plot(d2, res2.E_used_Wh, 'b-.', 'LineWidth', 2, 'DisplayName', 'Conv MPC');
plot(d3, res3.E_used_Wh, 'g-',  'LineWidth', 2.5, 'DisplayName', 'Bat-Aware MPC');
xlabel('Distance [m]'); ylabel('Energy [Wh]');
title('Cumulative Energy vs Distance'); legend('Location', 'northwest'); grid on;

sgtitle('DEMO 9: Controller Performance Comparison', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('  PID Energy:             %.1f Wh | Final SOC: %.1f%%\n', res1.summary.energy_Wh, res1.summary.final_soc);
fprintf('  Conv MPC Energy:        %.1f Wh | Final SOC: %.1f%%\n', res2.summary.energy_Wh, res2.summary.final_soc);
fprintf('  Battery-Aware MPC:      %.1f Wh | Final SOC: %.1f%%\n', res3.summary.energy_Wh, res3.summary.final_soc);
fprintf('  → Battery-Aware MPC saves %.1f%% energy vs PID!\n', (1 - res3.summary.energy_Wh/res1.summary.energy_Wh)*100);

fprintf('\n  ✓ Press any key for multi-SOC & environment results...\n');
pause;

%% ========================================================================
%  DEMO 10: MULTI-SOC ADAPTATION & ENVIRONMENTAL IMPACT
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 10: Multi-SOC Adaptation & Environmental Impact\n');
fprintf('══════════════════════════════════════════════════════════════\n');

res4 = all_results{4}; res5 = all_results{5}; res6 = all_results{6};
res7 = all_results{7}; res8 = all_results{8}; res9 = all_results{9}; res10 = all_results{10};

figure('Name', 'DEMO 10: SOC & Environment', 'Color', 'w', 'Position', [100 50 1000 600]);

subplot(1,2,1);
plot(res3.t, res3.SOC, 'g-',  'LineWidth', 2, 'DisplayName', '80% SOC'); hold on;
plot(res4.t, res4.SOC, 'b-',  'LineWidth', 2, 'DisplayName', '60% SOC');
plot(res5.t, res5.SOC, 'm-',  'LineWidth', 2, 'DisplayName', '40% SOC');
plot(res6.t, res6.SOC, 'r--', 'LineWidth', 2.5, 'DisplayName', '20% SOC (Preserved)');
yline(20, 'k:', 'Reserve Buffer', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('SOC [%]');
title('Battery-Aware MPC at Different Initial SOC'); legend('Location', 'best'); grid on;

subplot(1,2,2);
plot(res3.t,  res3.P,  'k-',  'LineWidth', 1.5, 'DisplayName', 'Calm Water'); hold on;
plot(res7.t,  res7.P,  'r--', 'LineWidth', 1.5, 'DisplayName', 'Opposing Current');
plot(res8.t,  res8.P,  'm-.', 'LineWidth', 1.5, 'DisplayName', 'Cross Current');
plot(res9.t,  res9.P,  'c:',  'LineWidth', 1.5, 'DisplayName', 'Wind 6 m/s');
plot(res10.t, res10.P, 'b-',  'LineWidth', 2, 'DisplayName', 'Combined Disturbances');
xlabel('Time [s]'); ylabel('Power [W]');
title('Power Under Environmental Disturbances'); legend('Location', 'best'); grid on;

sgtitle('DEMO 10: Adaptive Behaviour & Environmental Robustness', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('  ✓ Press any key for FINAL SUMMARY...\n');
pause;

%% ========================================================================
%  DEMO 11: FINAL COMPARISON SUMMARY
% =========================================================================
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  DEMO 11: FINAL QUANTITATIVE COMPARISON\n');
fprintf('══════════════════════════════════════════════════════════════\n');

figure('Name', 'DEMO 11: Final Comparison', 'Color', 'w', 'Position', [120 80 900 500]);
met_names = {'Energy [Wh]', 'Travel Time [s]', 'Speed RMSE ×100', 'CTE RMSE ×10'};
val_pid  = [res1.summary.energy_Wh, res1.summary.travel_time_s, res1.summary.rmse_u*100, res1.summary.rmse_cte*10];
val_conv = [res2.summary.energy_Wh, res2.summary.travel_time_s, res2.summary.rmse_u*100, res2.summary.rmse_cte*10];
val_ba   = [res3.summary.energy_Wh, res3.summary.travel_time_s, res3.summary.rmse_u*100, res3.summary.rmse_cte*10];

b = bar([val_pid; val_conv; val_ba]');
b(1).FaceColor = [0.85 0.25 0.25];
b(2).FaceColor = [0.25 0.45 0.85];
b(3).FaceColor = [0.20 0.75 0.30];
set(gca, 'XTickLabel', met_names, 'FontSize', 11);
legend('PID Controller', 'Conventional MPC', 'Battery-Aware MPC', 'Location', 'best', 'FontSize', 11);
title('FINAL: Quantitative Trade-Off Comparison', 'FontSize', 14);
ylabel('Metric Value'); grid on;

% Print summary table
fprintf('\n  ┌─────────────────────┬──────────┬──────────┬──────────┐\n');
fprintf('  │ Metric              │   PID    │ Conv MPC │ Bat MPC  │\n');
fprintf('  ├─────────────────────┼──────────┼──────────┼──────────┤\n');
fprintf('  │ Energy [Wh]         │ %8.1f │ %8.1f │ %8.1f │\n', res1.summary.energy_Wh, res2.summary.energy_Wh, res3.summary.energy_Wh);
fprintf('  │ Travel Time [s]     │ %8.1f │ %8.1f │ %8.1f │\n', res1.summary.travel_time_s, res2.summary.travel_time_s, res3.summary.travel_time_s);
fprintf('  │ Final SOC [%%]       │ %8.1f │ %8.1f │ %8.1f │\n', res1.summary.final_soc, res2.summary.final_soc, res3.summary.final_soc);
fprintf('  │ Speed RMSE [m/s]    │ %8.3f │ %8.3f │ %8.3f │\n', res1.summary.rmse_u, res2.summary.rmse_u, res3.summary.rmse_u);
fprintf('  │ Cross-Track RMSE [m]│ %8.3f │ %8.3f │ %8.3f │\n', res1.summary.rmse_cte, res2.summary.rmse_cte, res3.summary.rmse_cte);
fprintf('  │ Avg Power [W]       │ %8.1f │ %8.1f │ %8.1f │\n', res1.summary.avg_power_W, res2.summary.avg_power_W, res3.summary.avg_power_W);
fprintf('  │ Mission Complete    │ %8s │ %8s │ %8s │\n', string(res1.summary.is_complete), string(res2.summary.is_complete), string(res3.summary.is_complete));
fprintf('  └─────────────────────┴──────────┴──────────┴──────────┘\n');

fprintf('\n╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║              DEMONSTRATION COMPLETE!                           ║\n');
fprintf('║  All objectives and feedback points demonstrated.              ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n');

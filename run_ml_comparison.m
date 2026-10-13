%% =========================================================================
%  ML vs BASELINE CONTROLLER COMPARISON TABLE
%  ML Model: Adaptive Multi-Layer Perceptron (ANN) Neural Network
%  Optimizer: Adam Optimizer (Adaptive Moment Estimation)
%  Runs all 7 controllers and displays a side-by-side benchmark comparison
% =========================================================================
clc; clear; close all;

% Add all project folders to path
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(root_dir));

% Choose ML Method: 'NEURAL_NETWORK' (Option 1: ANN + Adam), 'BAYESIAN' (Option 3), or 'PSO'
ML_METHOD = 'NEURAL_NETWORK'; 

fprintf('╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║  ML vs BASELINE CONTROLLER COMPARISON                            ║\n');
fprintf('║  ML Model:   Adaptive Neural Network (Multi-Layer Perceptron)    ║\n');
fprintf('║  Optimizer:  Adam Optimizer (Adaptive Moment Estimation)         ║\n');
fprintf('║  Running 7 Controllers: PID, Conv MPC, Bat MPC,                  ║\n');
fprintf('║  PID+ML, PID+ML+Battery, MPC+ML, MPC+ML+Battery                  ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n\n');

%% Load Parameters
p = load_all_parameters();

%% Common scenario config
cfg.initial_soc    = 80.0;
cfg.plan_mode      = 'balanced';
cfg.t_max          = 1200.0;
cfg.enable_docking = false;

%% ===== BASELINE CONTROLLERS =====

% 1. PID Baseline
fprintf('[1/7] Running PID Baseline...\n');
cfg.controller_type = 'PID';
res_pid = run_simulation(p, cfg);

% 2. Conventional MPC
fprintf('[2/7] Running Conventional MPC...\n');
cfg.controller_type = 'CONV_MPC';
res_conv_mpc = run_simulation(p, cfg);

% 3. Battery-Aware MPC
fprintf('[3/7] Running Battery-Aware MPC...\n');
cfg.controller_type = 'BATTERY_AWARE_MPC';
res_bat_mpc = run_simulation(p, cfg);

%% ===== STEP 1: ML MODEL TRAINING (ANN + ADAM OPTIMIZER) =====
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  PHASE 1: TRAINING ADAPTIVE NEURAL NETWORK WITH ADAM OPTIMIZER\n');
fprintf('  Model: 6-32-16-13 Multi-Layer Perceptron (ReLU & Tanh)\n');
fprintf('  Optimizer: Adam (lr=0.005, beta1=0.9, beta2=0.999)\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

ann_opts.epochs     = 150;
ann_opts.batch_size = 32;
ann_opts.lr         = 0.005;
ann_opts.verbose    = true;

% Train the Neural Network model with Adam Optimizer
ann_model = train_neural_network(p, ann_opts);
p.ml_ann = ann_model;

% Query Neural Network for nominal operating point gains
[nom_pid_gains, nom_mpc_weights] = predict_neural_network([1.5, 1.5, 0.0, 0.0, 80.0, 0.0], p.ml_ann);
p.ml_pid.Kp_u   = nom_pid_gains(1);
p.ml_pid.Ki_u   = nom_pid_gains(2);
p.ml_pid.Kd_u   = nom_pid_gains(3);
p.ml_pid.Kp_psi = nom_pid_gains(4);
p.ml_pid.Ki_psi = nom_pid_gains(5);
p.ml_pid.Kd_psi = nom_pid_gains(6);

fprintf('  ✓ Nominal Neural Network PID Gains: Kp_u=%.1f, Ki_u=%.1f, Kd_u=%.1f, Kp_psi=%.1f, Ki_psi=%.1f, Kd_psi=%.1f\n', ...
    nom_pid_gains(1:6));
fprintf('  ✓ Nominal Neural Network MPC Weights: w_spd=%.2f, w_cte=%.2f, w_psi=%.2f, w_pwr=%.3f\n\n', ...
    nom_mpc_weights(1), nom_mpc_weights(2), nom_mpc_weights(3), nom_mpc_weights(5));

%% ===== STEP 2: RUN ML SIMULATIONS WITH ADAPTIVE NEURAL NETWORK =====
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  PHASE 2: RUNNING CLOSED-LOOP SIMULATIONS (REAL-TIME ML INFERENCE)\n');
fprintf('══════════════════════════════════════════════════════════════\n\n');

% 4. PID + ML (Real-Time Neural Network Inference)
fprintf('[4/7] Running PID + ML (Adaptive Neural Network)...\n');
cfg.controller_type = 'PID_ML';
res_pid_ml = run_simulation(p, cfg);

% 5. PID + ML + Battery Aware (Neural Network with SOC adaptation)
fprintf('[5/7] Running PID + ML + Battery Aware (Adaptive Neural Network)...\n');
cfg.controller_type = 'PID_ML_BATTERY';
res_pid_ml_bat = run_simulation(p, cfg);

% 6. MPC + ML (Neural Network-Tuned Weights)
fprintf('[6/7] Running MPC + ML (Neural Network-Tuned Weights)...\n');
p.mpc.weights.w_speed    = nom_mpc_weights(1);
p.mpc.weights.w_cte      = nom_mpc_weights(2);
p.mpc.weights.w_psi      = nom_mpc_weights(3);
p.mpc.weights.w_yaw_rate = nom_mpc_weights(4);
p.mpc.weights.w_rate_T   = nom_mpc_weights(6);
p.mpc.weights.w_rate_N   = nom_mpc_weights(7);
cfg.controller_type = 'MPC_ML';
res_mpc_ml = run_simulation(p, cfg);

% 7. MPC + ML + Battery Aware (Neural Network-Tuned Weights + Power Penalty)
fprintf('[7/7] Running MPC + ML + Battery Aware (Neural Network Weights)...\n');
p.mpc.weights.w_speed    = nom_mpc_weights(1);
p.mpc.weights.w_cte      = nom_mpc_weights(2);
p.mpc.weights.w_psi      = nom_mpc_weights(3);
p.mpc.weights.w_yaw_rate = nom_mpc_weights(4);
p.mpc.weights.w_power    = nom_mpc_weights(5);
p.mpc.weights.w_rate_T   = nom_mpc_weights(6);
p.mpc.weights.w_rate_N   = nom_mpc_weights(7);
cfg.controller_type = 'MPC_ML_BATTERY';
res_mpc_ml_bat = run_simulation(p, cfg);

fprintf('\n✓ All 7 simulations completed!\n\n');

%% ===== BUILD COMPARISON TABLE =====
names = {'PID', 'Conv MPC', 'Bat MPC', 'PID+ML (ANN)', 'PID+ML+Bat', 'MPC+ML (ANN)', 'MPC+ML+Bat'};
results = {res_pid, res_conv_mpc, res_bat_mpc, res_pid_ml, res_pid_ml_bat, res_mpc_ml, res_mpc_ml_bat};

% Extract metrics
energy     = zeros(1, 7);
travel_t   = zeros(1, 7);
final_soc  = zeros(1, 7);
rmse_u     = zeros(1, 7);
rmse_cte   = zeros(1, 7);
avg_power  = zeros(1, 7);
peak_power = zeros(1, 7);
complete   = cell(1, 7);

for i = 1:7
    s = results{i}.summary;
    energy(i)     = s.energy_Wh;
    travel_t(i)   = s.travel_time_s;
    final_soc(i)  = s.final_soc;
    rmse_u(i)     = s.rmse_u;
    rmse_cte(i)   = s.rmse_cte;
    avg_power(i)  = s.avg_power_W;
    peak_power(i) = s.peak_power_W;
    if s.is_complete
        complete{i} = 'true';
    else
        complete{i} = 'false';
    end
end

%% ===== PRINT TABLE =====
fprintf('┌─────────────────────┬─────────┬──────────┬─────────┬─────────┬────────────┬────────┬────────────┐\n');
fprintf('│ Metric              │   PID   │ Conv MPC │ Bat MPC │ PID+ML  │ PID+ML+Bat │ MPC+ML │ MPC+ML+Bat │\n');
fprintf('├─────────────────────┼─────────┼──────────┼─────────┼─────────┼────────────┼────────┼────────────┤\n');
fprintf('│ Energy [Wh]         │ %7.1f │ %8.1f │ %7.1f │ %7.1f │ %10.1f │ %6.1f │ %10.1f │\n', energy);
fprintf('│ Travel Time [s]     │ %7.1f │ %8.1f │ %7.1f │ %7.1f │ %10.1f │ %6.1f │ %10.1f │\n', travel_t);
fprintf('│ Final SOC [%%]       │ %7.1f │ %8.1f │ %7.1f │ %7.1f │ %10.1f │ %6.1f │ %10.1f │\n', final_soc);
fprintf('│ Speed RMSE [m/s]    │ %7.3f │ %8.3f │ %7.3f │ %7.3f │ %10.3f │ %6.3f │ %10.3f │\n', rmse_u);
fprintf('│ Cross-Track RMSE [m]│ %7.3f │ %8.3f │ %7.3f │ %7.3f │ %10.3f │ %6.3f │ %10.3f │\n', rmse_cte);
fprintf('│ Avg Power [W]       │ %7.1f │ %8.1f │ %7.1f │ %7.1f │ %10.1f │ %6.1f │ %10.1f │\n', avg_power);
fprintf('│ Peak Power [W]      │ %7.1f │ %8.1f │ %7.1f │ %7.1f │ %10.1f │ %6.1f │ %10.1f │\n', peak_power);
fprintf('│ Mission Complete    │ %7s │ %8s │ %7s │ %7s │ %10s │ %6s │ %10s │\n', complete{:});
fprintf('└─────────────────────┴─────────┴──────────┴─────────┴─────────┴────────────┴────────┴────────────┘\n');

%% ===== ENERGY SAVINGS CALCULATION =====
fprintf('\n══════════════════════════════════════════════════════════════\n');
fprintf('  ML (ANN + ADAM) IMPROVEMENT OVER BASELINES\n');
fprintf('══════════════════════════════════════════════════════════════\n');
fprintf('  PID+ML (ANN)     saves %.1f%% energy vs PID baseline\n', (1 - energy(4)/energy(1))*100);
fprintf('  PID+ML+Bat (ANN) saves %.1f%% energy vs PID baseline\n', (1 - energy(5)/energy(1))*100);
fprintf('  MPC+ML (ANN)     saves %.1f%% energy vs Conv MPC\n',     (1 - energy(6)/energy(2))*100);
fprintf('  MPC+ML+Bat (ANN) saves %.1f%% energy vs Bat MPC\n',      (1 - energy(7)/energy(3))*100);
fprintf('  Best ML controller preserves %.1f%% more SOC than PID\n', max(final_soc(4:7)) - final_soc(1));
fprintf('══════════════════════════════════════════════════════════════\n');

%% ===== PLOT RESULTS & ANN TRAINING CONVERGENCE =====
figure('Name', 'ML Neural Network vs Baseline Comparison', 'Color', 'w', 'Position', [40 40 1250 720]);

% Subplot 1: ANN Loss Convergence
subplot(2,4,1);
plot(1:length(ann_model.loss_history), ann_model.loss_history, 'b-', 'LineWidth', 2);
xlabel('Epoch'); ylabel('MSE Loss'); title('ANN Adam Training Loss'); grid on;

% Subplot 2: Energy comparison
subplot(2,4,2);
b1 = bar(energy);
b1.FaceColor = 'flat';
b1.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('Energy [Wh]'); title('Energy Consumption'); grid on;

% Subplot 3: Travel time
subplot(2,4,3);
b2 = bar(travel_t);
b2.FaceColor = 'flat';
b2.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('Time [s]'); title('Travel Time'); grid on;

% Subplot 4: Final SOC
subplot(2,4,4);
b3 = bar(final_soc);
b3.FaceColor = 'flat';
b3.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('SOC [%]'); title('Final Battery SOC'); grid on;
yline(20, 'r--', 'Reserve', 'LineWidth', 1.5);

% Subplot 5: Speed RMSE
subplot(2,4,5);
b4 = bar(rmse_u);
b4.FaceColor = 'flat';
b4.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('RMSE [m/s]'); title('Speed Tracking RMSE'); grid on;

% Subplot 6: Cross-Track RMSE
subplot(2,4,6);
b5 = bar(rmse_cte);
b5.FaceColor = 'flat';
b5.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('RMSE [m]'); title('Cross-Track Error RMSE'); grid on;

% Subplot 7: Avg Power
subplot(2,4,7);
b6 = bar(avg_power);
b6.FaceColor = 'flat';
b6.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('Power [W]'); title('Average Power Draw'); grid on;

% Subplot 8: Peak Power
subplot(2,4,8);
b7 = bar(peak_power);
b7.FaceColor = 'flat';
b7.CData = [0.85 0.25 0.25; 0.25 0.45 0.85; 0.20 0.75 0.30; ...
             0.95 0.50 0.20; 0.80 0.40 0.60; 0.40 0.70 0.90; 0.50 0.80 0.50];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 35, 'FontSize', 7.5);
ylabel('Peak Power [W]'); title('Peak Power Draw'); grid on;

sgtitle('Neural Network (ANN + Adam Optimizer) vs Baseline Controller Comparison', ...
    'FontSize', 14, 'FontWeight', 'bold');

%% ===== SAVE RESULTS TO CSV =====
T = table(energy', travel_t', final_soc', rmse_u', rmse_cte', avg_power', peak_power', ...
    'VariableNames', {'Energy_Wh', 'TravelTime_s', 'FinalSOC_pct', 'RMSE_Speed', 'RMSE_CTE', 'AvgPower_W', 'PeakPower_W'}, ...
    'RowNames', names');
writetable(T, 'ml_comparison_results.csv', 'WriteRowNames', true);
fprintf('\n✓ Results saved to ml_comparison_results.csv\n');

fprintf('\n╔══════════════════════════════════════════════════════════════════╗\n');
fprintf('║              ML COMPARISON COMPLETE!                            ║\n');
fprintf('╚══════════════════════════════════════════════════════════════════╝\n');

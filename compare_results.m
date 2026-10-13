
% ============================================
% MPC SIMULATION PERFORMANCE RESULTS
% ============================================

% Extract signals from Simulink output
speed_mpc = out.log_Speed;
soc_mpc = out.log_SOC;
u_ref_mpc = out.log_u_ref;

% Convert timeseries signals to numeric arrays
t_speed = speed_mpc.Time(:);
v = squeeze(speed_mpc.Data);
v = v(:);

t_ref = u_ref_mpc.Time(:);
ref = squeeze(u_ref_mpc.Data);
ref = ref(:);

t_soc = soc_mpc.Time(:);
soc = squeeze(soc_mpc.Data);
soc = soc(:);

% Align reference speed with measured speed
ref_aligned = interp1(t_ref, ref, t_speed, 'linear', 'extrap');

% Calculate tracking error
error = ref_aligned - v;

% Performance metrics
RMSE = sqrt(mean(error.^2));
MAE = mean(abs(error));

% Battery consumption
initial_SOC = soc(1);
final_SOC = soc(end);
SOC_consumed = initial_SOC - final_SOC;

% Display results
fprintf('\n====================================\n');
fprintf('       MPC SIMULATION RESULTS\n');
fprintf('====================================\n');

fprintf('Simulation duration : %.2f seconds\n', ...
    t_speed(end)-t_speed(1));
fprintf('Initial SOC         : %.2f %%\n', initial_SOC);
fprintf('Final SOC           : %.2f %%\n', final_SOC);
fprintf('SOC consumed        : %.2f percentage points\n', ...
    SOC_consumed);
fprintf('Speed tracking RMSE : %.4f m/s\n', RMSE);
fprintf('Speed tracking MAE  : %.4f m/s\n', MAE);

fprintf('====================================\n');

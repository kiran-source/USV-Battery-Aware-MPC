% ============================================
% PID SIMULATION PERFORMANCE RESULTS
% ============================================

% Extract signals from Simulink output
speed_pid = out.log_Speed_PID;
soc_pid = out.log_SOC_PID;
u_ref_pid = out.log_u_ref_PID;

% Convert timeseries signals to numeric arrays
t_speed = speed_pid.Time(:);
v = squeeze(speed_pid.Data);
v = v(:);

t_ref = u_ref_pid.Time(:);
ref = squeeze(u_ref_pid.Data);
ref = ref(:);

t_soc = soc_pid.Time(:);
soc = squeeze(soc_pid.Data);
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
fprintf('       PID SIMULATION RESULTS\n');
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
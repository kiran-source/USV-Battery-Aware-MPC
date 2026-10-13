function p = load_all_parameters()
    % LOAD_ALL_PARAMETERS Master loader aggregating all subsystem parameters
    
    p.usv         = USV_parameters();
    p.battery     = battery_parameters();
    p.mpc         = MPC_parameters();
    p.environment = environment_parameters();
    p.mission     = mission_parameters();
    
    %% ML-Optimized PID Gains & Neural Network Model
    % Default gains; can be adapted in real-time by Neural Network (ANN) or tuned by Bayesian Opt / PSO.
    p.ml_pid.Kp_u   = 150.0;   % Surge speed proportional gain
    p.ml_pid.Ki_u   = 10.0;    % Surge speed integral gain
    p.ml_pid.Kd_u   = 25.0;    % Surge speed derivative gain
    p.ml_pid.Kp_psi = 80.0;    % Heading proportional gain
    p.ml_pid.Ki_psi = 5.0;     % Heading integral gain
    p.ml_pid.Kd_psi = 20.0;    % Heading derivative gain

    % Load trained Neural Network Gain Scheduler if available
    ann_file = fullfile(fileparts(mfilename('fullpath')), '..', 'ml_tuning', 'ann_gain_scheduler_model.mat');
    if exist(ann_file, 'file')
        data_ann = load(ann_file);
        p.ml_ann = data_ann.ann_model;
    else
        p.ml_ann = [];
    end
end

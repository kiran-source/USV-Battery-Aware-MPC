function [T_cmd, N_cmd, state_pid] = PID_ML_controller(u_ref, psi_ref, u_meas, psi_meas, r_meas, dt, state_pid, p)
    % PID_ML_CONTROLLER PID Controller with Machine Learning (PSO) Optimized Gains
    %
    % Same structure as PID_controller but uses ML-tuned gains from p.ml_pid
    % instead of hand-tuned values.
    
    if isempty(state_pid)
        state_pid.int_eu    = 0.0;
        state_pid.prev_eu   = 0.0;
        state_pid.int_epsi  = 0.0;
        state_pid.prev_epsi = 0.0;
    end
    
    %% ML-OPTIMIZED GAINS (From Trained Neural Network ANN or Optimizer)
    e_psi = wrapToPi(psi_ref - psi_meas);
    e_u = u_ref - u_meas;

    if isfield(p, 'ml_ann') && ~isempty(p.ml_ann)
        % Real-time Adaptive Neural Network forward pass
        input_ann = [u_meas, u_ref, e_psi, r_meas, 80.0, 0.0];
        [ann_gains, ~] = predict_neural_network(input_ann, p.ml_ann);
        Kp_u   = ann_gains(1);
        Ki_u   = ann_gains(2);
        Kd_u   = ann_gains(3);
        Kp_psi = ann_gains(4);
        Ki_psi = ann_gains(5);
        Kd_psi = ann_gains(6);
    else
        Kp_u   = p.ml_pid.Kp_u;
        Ki_u   = p.ml_pid.Ki_u;
        Kd_u   = p.ml_pid.Kd_u;
        Kp_psi = p.ml_pid.Kp_psi;
        Ki_psi = p.ml_pid.Ki_psi;
        Kd_psi = p.ml_pid.Kd_psi;
    end
    
    %% SPEED LOOP
    e_u = u_ref - u_meas;
    state_pid.int_eu = state_pid.int_eu + e_u * dt;
    state_pid.int_eu = min(max(state_pid.int_eu, -5.0), 5.0);
    
    der_eu = (e_u - state_pid.prev_eu) / dt;
    state_pid.prev_eu = e_u;
    
    T_raw = Kp_u * e_u + Ki_u * state_pid.int_eu + Kd_u * der_eu;
    T_cmd = min(max(T_raw, p.usv.prop.T_min_tot), p.usv.prop.T_max_tot);
    
    %% HEADING LOOP
    e_psi = wrapToPi(psi_ref - psi_meas);
    state_pid.int_epsi = state_pid.int_epsi + e_psi * dt;
    state_pid.int_epsi = min(max(state_pid.int_epsi, -1.0), 1.0);
    
    N_raw = Kp_psi * e_psi + Ki_psi * state_pid.int_epsi - Kd_psi * r_meas;
    N_cmd = min(max(N_raw, -p.usv.prop.N_max), p.usv.prop.N_max);
end

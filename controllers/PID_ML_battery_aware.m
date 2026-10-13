function [T_cmd, N_cmd, state_pid] = PID_ML_battery_aware(u_ref, psi_ref, u_meas, psi_meas, r_meas, SOC_cur, dt, state_pid, p)
    % PID_ML_BATTERY_AWARE PID Controller with ML-Optimized Gains + Battery Awareness
    %
    % Combines PSO-optimized PID gains with SOC-based adaptive strategies:
    %   1. ML-tuned gains from PSO (p.ml_pid)
    %   2. Speed reference derating at low SOC
    %   3. Dynamic thrust capping based on SOC
    %   4. Adaptive gain scheduling based on SOC
    
    if isempty(state_pid)
        state_pid.int_eu    = 0.0;
        state_pid.prev_eu   = 0.0;
        state_pid.int_epsi  = 0.0;
        state_pid.prev_epsi = 0.0;
    end
    
    %% 1. SOC-BASED SCALING FACTOR
    SOC_economy = p.battery.soc_economy_pct;   % 50%
    SOC_reserve = p.battery.soc_reserve_pct;    % 20%
    
    if SOC_cur <= SOC_reserve
        soc_scale = 0.3;
    elseif SOC_cur <= SOC_economy
        soc_scale = 0.3 + 0.7 * ((SOC_cur - SOC_reserve) / (SOC_economy - SOC_reserve));
    else
        soc_scale = 1.0;
    end
    
    u_ref_derated = u_ref * soc_scale;
    
    %% 2. ML-OPTIMIZED GAINS WITH SOC ADAPTATION (From Trained Neural Network ANN or Tuner)
    e_psi = wrapToPi(psi_ref - psi_meas);
    e_u = u_ref_derated - u_meas;

    if isfield(p, 'ml_ann') && ~isempty(p.ml_ann)
        % Real-time Adaptive Neural Network forward pass with battery SOC
        input_ann = [u_meas, u_ref_derated, e_psi, r_meas, SOC_cur, 0.0];
        [ann_gains, ~] = predict_neural_network(input_ann, p.ml_ann);
        Kp_u   = ann_gains(1) * soc_scale;
        Ki_u   = ann_gains(2) * soc_scale;
        Kd_u   = ann_gains(3);
        Kp_psi = ann_gains(4);
        Ki_psi = ann_gains(5);
        Kd_psi = ann_gains(6);
    else
        Kp_u   = p.ml_pid.Kp_u * soc_scale;
        Ki_u   = p.ml_pid.Ki_u * soc_scale;
        Kd_u   = p.ml_pid.Kd_u;
        Kp_psi = p.ml_pid.Kp_psi;
        Ki_psi = p.ml_pid.Ki_psi;
        Kd_psi = p.ml_pid.Kd_psi;
    end
    
    %% 3. SPEED LOOP (ML-tuned + SOC-derated)
    e_u = u_ref_derated - u_meas;
    state_pid.int_eu = state_pid.int_eu + e_u * dt;
    state_pid.int_eu = min(max(state_pid.int_eu, -5.0), 5.0);
    
    der_eu = (e_u - state_pid.prev_eu) / dt;
    state_pid.prev_eu = e_u;
    
    T_raw = Kp_u * e_u + Ki_u * state_pid.int_eu + Kd_u * der_eu;
    
    %% 4. BATTERY-AWARE THRUST CAPPING
    if SOC_cur <= SOC_reserve
        T_max_allowed = 80.0;
    elseif SOC_cur <= SOC_economy
        T_max_allowed = 80.0 + (p.usv.prop.T_max_tot - 80.0) * ...
            ((SOC_cur - SOC_reserve) / (SOC_economy - SOC_reserve));
    else
        T_max_allowed = p.usv.prop.T_max_tot;
    end
    
    T_cmd = min(max(T_raw, p.usv.prop.T_min_tot), T_max_allowed);
    
    %% 5. HEADING LOOP (ML-tuned, unchanged for safety)
    e_psi = wrapToPi(psi_ref - psi_meas);
    state_pid.int_epsi = state_pid.int_epsi + e_psi * dt;
    state_pid.int_epsi = min(max(state_pid.int_epsi, -1.0), 1.0);
    
    N_raw = Kp_psi * e_psi + Ki_psi * state_pid.int_epsi - Kd_psi * r_meas;
    N_cmd = min(max(N_raw, -p.usv.prop.N_max), p.usv.prop.N_max);
end

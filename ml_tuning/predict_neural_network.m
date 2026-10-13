function [pid_gains, mpc_weights] = predict_neural_network(input_vec, ann_model)
    % PREDICT_NEURAL_NETWORK Real-time forward inference of the trained ANN model
    %
    % Inputs:
    %   input_vec - [u_meas, u_ref, e_psi, r_meas, SOC, e_y] (1 x 6 vector)
    %   ann_model - Struct containing trained weights and scaling parameters
    %
    % Outputs:
    %   pid_gains   - [Kp_u, Ki_u, Kd_u, Kp_psi, Ki_psi, Kd_psi] (1 x 6 vector)
    %   mpc_weights - [w_speed, w_cte, w_psi, w_yaw, w_power, w_rate_T, w_rate_N] (1 x 7 vector)

    % Default fallback in case model is empty
    if isempty(ann_model) || ~isfield(ann_model, 'W1')
        pid_gains = [150.0, 10.0, 25.0, 80.0, 5.0, 20.0];
        mpc_weights = [80.0, 120.0, 100.0, 15.0, 0.08, 0.05, 0.10];
        return;
    end

    % Ensure row vector [1 x 6]
    if length(input_vec) < 6
        % If e_y or SOC not provided, pad defaults
        inp = zeros(1, 6);
        inp(1:length(input_vec)) = input_vec;
        if length(input_vec) < 5, inp(5) = 80.0; end % Default SOC
        input_vec = inp;
    end
    x = input_vec(:)';

    % Normalize input with trained z-scores
    x_norm = (x - ann_model.mu_X) ./ ann_model.sig_X;

    % Forward propagation through Layer 1 (ReLU)
    z1 = x_norm * ann_model.W1 + ann_model.b1;
    a1 = max(0, z1);

    % Forward propagation through Layer 2 (Tanh)
    z2 = a1 * ann_model.W2 + ann_model.b2;
    a2 = tanh(z2);

    % Linear output layer
    y_norm = a2 * ann_model.W3 + ann_model.b3;

    % Denormalize back to physical scale
    y_pred = ann_model.mu_Y + y_norm .* ann_model.sig_Y;

    % Ensure physical positivity constraints
    pid_gains = max([10.0, 0.1, 1.0, 5.0, 0.1, 1.0], y_pred(1:6));
    mpc_weights = max([0.1, 0.1, 0.01, 0.001, 0.001, 0.0001, 0.0001], y_pred(7:13));

end

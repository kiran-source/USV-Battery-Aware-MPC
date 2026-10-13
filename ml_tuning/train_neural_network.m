function ann_model = train_neural_network(p, options)
    % TRAIN_NEURAL_NETWORK Trains an Artificial Neural Network (ANN) with Adam Optimizer
    %
    % Architecture:
    %   - Model: Multi-Layer Perceptron (MLP / Feedforward Neural Network)
    %   - Input Layer (6 nodes): [u_meas, u_ref, e_psi, r_meas, SOC, e_y]
    %   - Hidden Layer 1: 32 Neurons with ReLU activation
    %   - Hidden Layer 2: 16 Neurons with Tanh activation
    %   - Output Layer (13 nodes):
    %       * 6 PID gains: [Kp_u, Ki_u, Kd_u, Kp_psi, Ki_psi, Kd_psi]
    %       * 7 MPC weights: [w_speed, w_cte, w_psi, w_yaw, w_power, w_rate_T, w_rate_N]
    %   - Optimizer: Adam (Adaptive Moment Estimation)
    %       * Learning rate alpha = 0.005
    %       * Beta1 = 0.9, Beta2 = 0.999, Epsilon = 1e-8
    %
    % Inputs:
    %   p       - Parameter structure from load_all_parameters()
    %   options - Configuration struct:
    %               .epochs     (default 150)
    %               .batch_size (default 32)
    %               .lr         (default 0.005)
    %               .verbose    (default true)
    %
    % Outputs:
    %   ann_model - Struct containing trained weight matrices, biases, and normalization parameters.

    %% 1. Default Options Setup
    if nargin < 2, options = struct(); end
    epochs     = getfield_default(options, 'epochs', 150);
    batch_size = getfield_default(options, 'batch_size', 32);
    lr         = getfield_default(options, 'lr', 0.005);
    verbose    = getfield_default(options, 'verbose', true);

    %% 2. GENERATE TRAINING DATASET ACROSS OPERATING ENVELOPE
    % Inputs: [u_meas (0..2.5), u_ref (0..2.5), e_psi (-pi..pi), r (-0.5..0.5), SOC (10..100), e_y (-10..10)]
    N_samples = 1200;
    rng(42); % For exact reproducibility

    u_meas_data = 0.1 + 2.4 * rand(N_samples, 1);
    u_ref_data  = 0.5 + 2.0 * rand(N_samples, 1);
    e_psi_data  = -pi + 2*pi * rand(N_samples, 1);
    r_meas_data = -0.5 + 1.0 * rand(N_samples, 1);
    soc_data    = 10.0 + 90.0 * rand(N_samples, 1);
    e_y_data    = -10.0 + 20.0 * rand(N_samples, 1);

    X = [u_meas_data, u_ref_data, e_psi_data, r_meas_data, soc_data, e_y_data];

    % Ground truth target generation based on multi-objective control laws
    % Speed loop targets: Adapt gain based on speed error & battery SOC
    e_u = abs(u_ref_data - u_meas_data);
    soc_factor = min(1.0, max(0.3, (soc_data - 20.0) / 60.0));

    % Target PID Gains
    Kp_u_target   = (120.0 + 60.0 * tanh(2.0 * e_u)) .* soc_factor;
    Ki_u_target   = (8.0 + 4.0 * tanh(e_u)) .* soc_factor;
    Kd_u_target   = 20.0 + 10.0 * (1.0 - soc_factor);
    Kp_psi_target = 80.0 + 40.0 * abs(sin(e_psi_data / 2)) + 5.0 * abs(e_y_data);
    Ki_psi_target = 5.0 + 3.0 * abs(sin(e_psi_data));
    Kd_psi_target = 25.0 + 15.0 * abs(r_meas_data);

    % Target MPC Weights (Calibrated for high-precision path & speed tracking)
    w_spd_target = max(20.0, 100.0 - 15.0 * min(5.0, abs(e_y_data)));
    w_cte_target = 120.0 + 150.0 * min(5.0, abs(e_y_data));
    w_psi_target = 400.0 + 300.0 * min(pi, abs(e_psi_data));
    w_yaw_target = 5.0 * ones(N_samples, 1);
    w_pwr_target = 0.08 + 0.04 * (1.0 - soc_factor);
    w_rT_target  = 0.05 * ones(N_samples, 1);
    w_rN_target  = 0.10 * ones(N_samples, 1);

    Y = [Kp_u_target, Ki_u_target, Kd_u_target, Kp_psi_target, Ki_psi_target, Kd_psi_target, ...
         w_spd_target, w_cte_target, w_psi_target, w_yaw_target, w_pwr_target, w_rT_target, w_rN_target];

    %% 3. DATA NORMALIZATION (Z-Score)
    mu_X = mean(X, 1);
    sig_X = std(X, 0, 1);
    sig_X(sig_X < 1e-6) = 1.0;
    X_norm = (X - mu_X) ./ sig_X;

    mu_Y = mean(Y, 1);
    sig_Y = std(Y, 0, 1);
    sig_Y(sig_Y < 1e-6) = 1.0;
    Y_norm = (Y - mu_Y) ./ sig_Y;

    %% 4. INITIALIZE NEURAL NETWORK WEIGHTS (He / Xavier Initialization)
    n_in  = 6;
    n_h1  = 32;
    n_h2  = 16;
    n_out = 13;

    W1 = randn(n_in, n_h1) * sqrt(2.0 / n_in);
    b1 = zeros(1, n_h1);

    W2 = randn(n_h1, n_h2) * sqrt(2.0 / n_h1);
    b2 = zeros(1, n_h2);

    W3 = randn(n_h2, n_out) * sqrt(1.0 / n_h2);
    b3 = zeros(1, n_out);

    % Adam Optimizer States
    mW1 = zeros(size(W1)); vW1 = zeros(size(W1));
    mb1 = zeros(size(b1)); vb1 = zeros(size(b1));
    mW2 = zeros(size(W2)); vW2 = zeros(size(W2));
    mb2 = zeros(size(b2)); vb2 = zeros(size(b2));
    mW3 = zeros(size(W3)); vW3 = zeros(size(W3));
    mb3 = zeros(size(b3)); vb3 = zeros(size(b3));

    beta1 = 0.9;
    beta2 = 0.999;
    eps_adam = 1e-8;
    t_step = 0;

    loss_history = zeros(epochs, 1);

    if verbose
        fprintf('  [Neural Network Training] Architecture: 6 -> 32 (ReLU) -> 16 (Tanh) -> 13\n');
        fprintf('  [Optimizer: Adam] Learning Rate: %.4f, Epochs: %d, Batch Size: %d\n', lr, epochs, batch_size);
    end

    %% 5. ADAM TRAINING LOOP (Mini-Batch Gradient Descent)
    n_batches = floor(N_samples / batch_size);

    for epoch = 1:epochs
        idx_shuffle = randperm(N_samples);
        epoch_loss = 0.0;

        for b = 1:n_batches
            t_step = t_step + 1;
            batch_idx = idx_shuffle((b-1)*batch_size + 1 : b*batch_size);

            xb = X_norm(batch_idx, :);
            yb = Y_norm(batch_idx, :);

            % --- FORWARD PASS ---
            z1 = xb * W1 + b1;
            a1 = max(0, z1); % ReLU activation

            z2 = a1 * W2 + b2;
            a2 = tanh(z2);   % Tanh activation

            y_pred = a2 * W3 + b3; % Linear output

            % Compute Loss (MSE)
            err = y_pred - yb;
            batch_loss = mean(sum(err .^ 2, 2));
            epoch_loss = epoch_loss + batch_loss;

            % --- BACKPROPAGATION ---
            d_out = (2.0 / batch_size) * err;

            dW3 = a2' * d_out;
            db3 = sum(d_out, 1);

            d_a2 = d_out * W3';
            d_z2 = d_a2 .* (1.0 - a2 .^ 2); % Derivative of Tanh

            dW2 = a1' * d_z2;
            db2 = sum(d_z2, 1);

            d_a1 = d_z2 * W2';
            d_z1 = d_a1 .* (z1 > 0); % Derivative of ReLU

            dW1 = xb' * d_z1;
            db1 = sum(d_z1, 1);

            % --- ADAM OPTIMIZER STEP ---
            % Layer 1
            [W1, mW1, vW1] = adam_step(W1, dW1, mW1, vW1, lr, beta1, beta2, eps_adam, t_step);
            [b1, mb1, vb1] = adam_step(b1, db1, mb1, vb1, lr, beta1, beta2, eps_adam, t_step);
            % Layer 2
            [W2, mW2, vW2] = adam_step(W2, dW2, mW2, vW2, lr, beta1, beta2, eps_adam, t_step);
            [b2, mb2, vb2] = adam_step(b2, db2, mb2, vb2, lr, beta1, beta2, eps_adam, t_step);
            % Layer 3
            [W3, mW3, vW3] = adam_step(W3, dW3, mW3, vW3, lr, beta1, beta2, eps_adam, t_step);
            [b3, mb3, vb3] = adam_step(b3, db3, mb3, vb3, lr, beta1, beta2, eps_adam, t_step);
        end

        loss_history(epoch) = epoch_loss / n_batches;

        if verbose && (mod(epoch, 30) == 0 || epoch == 1 || epoch == epochs)
            fprintf('  Epoch %3d/%d | MSE Loss: %.6f\n', epoch, epochs, loss_history(epoch));
        end
    end

    %% 6. PACK TRAINED MODEL
    ann_model.W1 = W1;
    ann_model.b1 = b1;
    ann_model.W2 = W2;
    ann_model.b2 = b2;
    ann_model.W3 = W3;
    ann_model.b3 = b3;
    ann_model.mu_X  = mu_X;
    ann_model.sig_X = sig_X;
    ann_model.mu_Y  = mu_Y;
    ann_model.sig_Y = sig_Y;
    ann_model.loss_history = loss_history;
    ann_model.trained_date = datestr(now);

    % Save model artifact
    save_path = fullfile(fileparts(mfilename('fullpath')), 'ann_gain_scheduler_model.mat');
    save(save_path, 'ann_model');

    if verbose
        fprintf('  ✓ Neural Network training complete! Saved to ann_gain_scheduler_model.mat\n');
    end
end

function [param, m, v] = adam_step(param, grad, m, v, lr, beta1, beta2, eps_val, t)
    m = beta1 * m + (1.0 - beta1) * grad;
    v = beta2 * v + (1.0 - beta2) * (grad .^ 2);
    m_hat = m / (1.0 - beta1^t);
    v_hat = v / (1.0 - beta2^t);
    param = param - (lr ./ (sqrt(v_hat) + eps_val)) .* m_hat;
end

function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

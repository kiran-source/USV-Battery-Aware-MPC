function [T_opt, pred_speed, pred_thrust] = mpc_controller(u_meas, u_ref, T_prev, params)
    % MPC_CONTROLLER Solves constrained receding-horizon speed control for USV
    %
    % Inputs:
    %   u_meas : Current USV speed [m/s]
    %   u_ref  : Desired target speed [m/s]
    %   T_prev : Thrust applied in previous time step [N]
    %   params : Parameter struct from parameters()
    %
    % Outputs:
    %   T_opt       : Optimal thrust command for the current step [N]
    %   pred_speed  : Predicted speed trajectory over prediction horizon [m/s]
    %   pred_thrust : Predicted thrust trajectory over control horizon [N]

    %% Extract configuration
    Ts    = params.mpc.Ts;
    Np    = params.mpc.Np;
    Nc    = params.mpc.Nc;
    m     = params.usv.m;
    Cd    = params.usv.Cd;
    T_min = params.usv.T_min;
    T_max = params.usv.T_max;
    u_max = params.usv.u_max;
    w1    = params.mpc.w_speed;
    w2    = params.mpc.w_power;
    w3    = params.mpc.w_rate;

    %% Linearization around current speed
    u0 = max(u_meas, 0.05); % avoid singular linearization at dead stop
    A_c = -2 * Cd * u0 / m;
    B_c = 1 / m;
    d_c = (Cd * u0^2) / m;   % linearization offset

    % Discrete-time affine model: u(k+1) = A_d * u(k) + B_d * T(k) + d_d
    A_d = max(1.0 + A_c * Ts, 0.01);
    B_d = B_c * Ts;
    d_d = d_c * Ts;

    %% Formulate Prediction Matrices
    % U_pred = Sx * u_meas + Su * T_vec + Sd
    Sx = zeros(Np, 1);
    Su = zeros(Np, Nc);
    Sd = zeros(Np, 1);

    cur_A_pow = 1.0;
    sum_d = 0.0;

    for i = 1:Np
        cur_A_pow = cur_A_pow * A_d;
        Sx(i) = cur_A_pow;

        % Cumulative affine disturbance term
        sum_d = sum_d * A_d + d_d;
        Sd(i) = sum_d;

        % Input influence matrix Su
        for j = 1:min(i, Nc)
            Su(i, j) = (A_d^(i - j)) * B_d;
        end
        if i > Nc
            % Input is held constant after control horizon Nc
            for j = 1:Nc-1
                % already filled
            end
            % last input held for remaining steps
            Su(i, Nc) = sum(A_d.^(0:(i-Nc))) * B_d;
        end
    end

    %% Build Quadratic Cost Function: 0.5 * T'*H*T + f'*T
    % Tracking error cost: w1 * || Su*T + (Sx*u_meas + Sd - u_ref) ||^2
    err_0 = (Sx * u_meas + Sd) - (u_ref * ones(Np, 1));
    H_track = 2 * w1 * (Su' * Su);
    f_track = 2 * w1 * (Su' * err_0);

    % Power penalty: w2 * sum(T_k * u_pred_approx)
    % Linearized power cost: w2 * u0 * sum(T_k)
    f_power = w2 * u0 * ones(Nc, 1);

    % Rate of change cost: w3 * || D * T - d0 ||^2
    % D * T = [T(1) - T_prev; T(2) - T(1); ...]
    D = eye(Nc);
    for k = 2:Nc
        D(k, k-1) = -1.0;
    end
    d0 = zeros(Nc, 1);
    d0(1) = T_prev;

    H_rate = 2 * w3 * (D' * D);
    f_rate = -2 * w3 * (D' * d0);

    % Combined Hessian and gradient
    H = H_track + H_rate + 1e-4 * eye(Nc); % Regularization for numerical conditioning
    f = f_track + f_power + f_rate;

    %% Box-Constrained QP Solver (Fast Projected Gradient with Armijo line-search)
    % Optimal thrust vector T within [T_min, T_max]
    lb = T_min * ones(Nc, 1);
    ub = T_max * ones(Nc, 1);

    % Initial guess: warm start with previous thrust
    T_curr = max(min(T_prev * ones(Nc, 1), ub), lb);

    % Projected gradient iterations
    max_iter = 40;
    step_size = 1.0 / (max(eig(H)) + 1e-3);

    for iter = 1:max_iter
        grad = H * T_curr + f;
        T_new = max(min(T_curr - step_size * grad, ub), lb);
        
        % Stopping criterion
        if norm(T_new - T_curr) < 1e-3
            T_curr = T_new;
            break;
        end
        T_curr = T_new;
    end

    T_opt = T_curr(1);

    % Output predicted trajectories for inspection/dashboard
    pred_thrust = T_curr;
    pred_speed = Sx * u_meas + Su * T_curr + Sd;
    pred_speed = max(min(pred_speed, u_max), 0.0);
end

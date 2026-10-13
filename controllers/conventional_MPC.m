function [T_cmd, N_cmd, diagnostics] = conventional_MPC(eta, nu, u_ref, psi_ref, e_y, T_prev, N_prev, p)
    % CONVENTIONAL_MPC 3-DOF USV Model Predictive Controller (Speed & LOS Heading Tracking)
    %
    % Inputs:
    %   eta     - [x; y; psi] - Global NED pose
    %   nu      - [u; v; r]   - Body velocities (surge, sway, yaw rate)
    %   u_ref   - Desired surge velocity [m/s]
    %   psi_ref - Desired LOS heading angle [rad]
    %   e_y     - Cross-track error [m]
    %   T_prev  - Previous thrust command [N]
    %   N_prev  - Previous yaw moment [N*m]
    %   p       - Subsystem parameter struct
    %
    % Outputs:
    %   T_cmd       - Optimal surge thrust command [N]
    %   N_cmd       - Optimal yaw moment command [N*m]
    %   diagnostics - Optimization telemetry struct

    Ts = p.mpc.Ts;
    Np = p.mpc.Np;
    Nc = p.mpc.Nc;

    u_meas   = nu(1);
    r_meas   = nu(3);
    psi_meas = eta(3);

    u0 = max(abs(u_meas), 0.10);

    %% 1. VESSEL LINEARIZED MATRICES
    m_u = max(p.usv.m - p.usv.X_udot, 1.0);
    m_r = max(p.usv.Iz - p.usv.N_rdot, 1.0);

    a_u = -(abs(p.usv.Xu) + 2 * abs(p.usv.Xuu) * u0) / m_u;
    b_u = 1.0 / m_u;
    d_u = abs(p.usv.Xuu) * (u0^2) / m_u;

    a_r = -abs(p.usv.Nr) / m_r;
    b_r = 1.0 / m_r;

    %% 2. ERROR-STATE DYNAMICS: x = [u - u_ref; r; wrapToPi(psi - psi_ref); e_y]
    Ad = [
        1 + a_u * Ts,   0,              0,              0;
        0,              1 + a_r * Ts,   0,              0;
        0,              Ts,             1,              0;
        0,              0,              u_ref * Ts,     1
    ];

    Bd = [
        b_u * Ts,   0;
        0,          b_r * Ts;
        0,          0;
        0,          0
    ];

    Dd = [(d_u + a_u * u_ref) * Ts; 0; 0; 0];

    Nx = 4;
    Nu = 2;

    %% 3. PREDICTION MATRICES OVER HORIZONS (Np, Nc)
    Sx = zeros(Nx * Np, Nx);
    Su = zeros(Nx * Np, Nu * Nc);
    Sd = zeros(Nx * Np, 1);

    A_pow = eye(Nx);
    for i = 1:Np
        A_pow = A_pow * Ad;
        rows = (i-1)*Nx + (1:Nx);
        Sx(rows, :) = A_pow;

        d_accum = zeros(Nx, 1);
        for k = 0:(i-1)
            d_accum = d_accum + (Ad^k) * Dd;
        end
        Sd(rows) = d_accum;

        for j = 1:Nc
            if i >= j
                cols = (j-1)*Nu + (1:Nu);
                Su(rows, cols) = (Ad^(i-j)) * Bd;
            end
        end
    end

    %% 4. CURRENT STATE & REFERENCE
    e_u0   = u_meas - u_ref;
    e_psi0 = wrapToPi(psi_meas - psi_ref);
    x0 = [e_u0; r_meas; e_psi0; e_y];

    X_ref = zeros(Nx * Np, 1); % Errors target 0

    %% 5. QUADRATIC TRACKING & SLEW-RATE MATRICES
    w_spd = max(p.mpc.weights.w_speed, 10.0);
    w_yaw = max(p.mpc.weights.w_yaw_rate, 5.0);
    w_psi = max(p.mpc.weights.w_psi, 20.0);
    w_cte = max(p.mpc.weights.w_cte, 10.0);

    w_rate_T = max(p.mpc.weights.w_rate_T, 0.01);
    w_rate_N = max(p.mpc.weights.w_rate_N, 0.01);

    Q_block = diag([w_spd, w_yaw, w_psi, w_cte]);
    Q_bar = kron(eye(Np), Q_block);

    R_rate_block = diag([w_rate_T, w_rate_N]);
    D_mat = eye(Nu * Nc) - [zeros(Nu, Nu * Nc); eye(Nu * (Nc-1)), zeros(Nu * (Nc-1), Nu)];
    R_bar = D_mat' * kron(eye(Nc), R_rate_block) * D_mat;
    H = 2 * (Su' * Q_bar * Su + R_bar) + 1e-4 * eye(Nu * Nc);

    e0_vec = Sx * x0 + Sd - X_ref;
    u_prev_vec = zeros(Nu * Nc, 1);
    u_prev_vec(1:2) = [T_prev; N_prev];
    d0_vec = [u_prev_vec(1:2); zeros(Nu * (Nc-1), 1)];

    f = 2 * Su' * Q_bar * e0_vec - 2 * D_mat' * kron(eye(Nc), R_rate_block) * d0_vec;

    %% 6. BOUNDS & CONSTRAINTS
    T_min = p.mpc.bounds.T_min;
    T_max = p.mpc.bounds.T_max;
    N_max = p.mpc.bounds.N_max;
    delta_T_max = p.mpc.bounds.delta_T_max;
    delta_N_max = p.mpc.bounds.delta_N_max;

    lb = repmat([T_min; -N_max], Nc, 1);
    ub = repmat([T_max;  N_max], Nc, 1);

    %% 7. SOLVE QP VIA PROJECTED ACTIVE-SET GRADIENT
    U_opt = repmat([T_prev; N_prev], Nc, 1);
    alpha_step = 1.0 / (max(eig(H)) + 1e-3);

    for iter = 1:p.mpc.solver.max_iter
        grad = H * U_opt + f;
        U_new = min(max(U_opt - alpha_step * grad, lb), ub);

        % Enforce dynamic rate constraints
        for j = 1:Nc
            idx_T = (j-1)*Nu + 1;
            idx_N = (j-1)*Nu + 2;
            if j == 1
                U_new(idx_T) = min(max(U_new(idx_T), T_prev - delta_T_max), T_prev + delta_T_max);
                U_new(idx_N) = min(max(U_new(idx_N), N_prev - delta_N_max), N_prev + delta_N_max);
            else
                prev_T = U_new((j-2)*Nu + 1);
                prev_N = U_new((j-2)*Nu + 2);
                U_new(idx_T) = min(max(U_new(idx_T), prev_T - delta_T_max), prev_T + delta_T_max);
                U_new(idx_N) = min(max(U_new(idx_N), prev_N - delta_N_max), prev_N + delta_N_max);
            end
        end

        if norm(U_new - U_opt) < p.mpc.solver.tol
            break;
        end
        U_opt = U_new;
    end

    T_cmd = min(max(U_opt(1), T_min), T_max);
    N_cmd = min(max(U_opt(2), -N_max), N_max);

    diagnostics.iterations = iter;
    diagnostics.cost = 0.5 * U_opt' * H * U_opt + f' * U_opt;
    diagnostics.mode = 'CONVENTIONAL_MPC';
    diagnostics.e_psi = e_psi0;
    diagnostics.e_u = e_u0;
end
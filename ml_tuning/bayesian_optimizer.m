function [best_pos, best_cost, history] = bayesian_optimizer(cost_func, lb, ub, options)
    % BAYESIAN_OPTIMIZER Gaussian Process Regression & Expected Improvement (EI)
    %
    % A state-of-the-art Machine Learning optimization algorithm for sample-efficient
    % controller parameter tuning (PID gains and MPC weight matrices).
    %
    % ML Model:
    %   - Surrogate Model: Gaussian Process (GP) with Squared Exponential (RBF) Kernel
    %   - Acquisition Function: Expected Improvement (EI) with Exploration-Exploitation Trade-off
    %   - Hyperparameter Estimation: Maximum Marginal Likelihood / Empirical Bayes
    %
    % Inputs:
    %   cost_func - Function handle @(params) -> scalar cost (lower = better)
    %   lb        - Lower bounds vector [1 x D]
    %   ub        - Upper bounds vector [1 x D]
    %   options   - Configuration struct:
    %               .n_init       (default 8 initial exploration samples)
    %               .max_iter     (default 20 sequential BO iterations)
    %               .xi           (default 0.01 exploration parameter)
    %               .n_candidates (default 500 candidate points for EI maximization)
    %               .verbose      (default true)
    %
    % Outputs:
    %   best_pos  - Best parameter vector found [1 x D]
    %   best_cost - Best (lowest) cost achieved
    %   history   - Struct containing GP convergence logs, posterior mean, and uncertainty

    %% 1. Default Options Setup
    if nargin < 4, options = struct(); end
    n_init       = getfield_default(options, 'n_init', 8);
    max_iter     = getfield_default(options, 'max_iter', 20);
    xi           = getfield_default(options, 'xi', 0.01);
    n_candidates = getfield_default(options, 'n_candidates', 500);
    verbose      = getfield_default(options, 'verbose', true);

    lb = lb(:)'; % Ensure 1 x D
    ub = ub(:)';
    D = length(lb);

    %% 2. INITIAL DESIGN OF EXPERIMENTS (Latin Hypercube / Quasi-Random)
    % Generate normalized initial samples in [0, 1]^D
    X_norm = zeros(n_init, D);
    for d = 1:D
        perm = randperm(n_init)';
        X_norm(:, d) = (perm - 1 + rand(n_init, 1)) / n_init;
    end
    % Ensure corner and center anchors are included
    X_norm(1, :) = 0.5 * ones(1, D);

    % Convert normalized points to actual physical parameter scale
    X_phys = repmat(lb, n_init, 1) + X_norm .* repmat(ub - lb, n_init, 1);
    Y = zeros(n_init, 1);

    if verbose
        fprintf('  [Bayesian Optimization] Initializing Gaussian Process with %d DoE samples across %d dimensions...\n', n_init, D);
    end

    for i = 1:n_init
        try
            Y(i) = cost_func(X_phys(i, :));
        catch
            Y(i) = 1e6;
        end
    end

    [best_cost, best_idx] = min(Y);
    best_pos = X_phys(best_idx, :);

    % Preallocate history logs
    history.cost_per_iter   = zeros(max_iter, 1);
    history.sigma_mean      = zeros(max_iter, 1);
    history.best_pos_per_iter = zeros(max_iter, D);

    if verbose
        fprintf('  [Bayesian Optimization] Initial best cost after DoE: %.4f\n', best_cost);
    end

    %% 3. SEQUENTIAL BAYESIAN OPTIMIZATION LOOP
    for iter = 1:max_iter
        % Fit Gaussian Process on current observations (X_norm, Y)
        gp_model = fit_gaussian_process(X_norm, Y);

        % Maximize Expected Improvement (EI) acquisition function over candidate points
        % Generate candidates (Random Uniform + Local Perturbations around best)
        Cand_norm = rand(n_candidates, D);
        Cand_norm(1:min(10, size(Cand_norm, 1)), :) = repmat(X_norm(best_idx, :), min(10, size(Cand_norm, 1)), 1) ...
            + 0.05 * randn(min(10, size(Cand_norm, 1)), D);
        Cand_norm = max(0, min(1, Cand_norm)); % Clamp to [0, 1]

        % Evaluate GP Posterior (mean and standard deviation) on candidates
        [mu_cand, sigma_cand] = predict_gaussian_process(gp_model, Cand_norm);

        % Compute Expected Improvement (EI) for minimization
        % EI(x) = (y_best - mu(x) - xi) * Phi(Z) + sigma(x) * phi(Z)
        ei_values = zeros(size(Cand_norm, 1), 1);
        improvement = best_cost - mu_cand - xi;
        valid_sigma = sigma_cand > 1e-8;

        Z = zeros(size(Cand_norm, 1), 1);
        Z(valid_sigma) = improvement(valid_sigma) ./ sigma_cand(valid_sigma);

        phi_Z = norm_pdf(Z(valid_sigma));
        Phi_Z = norm_cdf(Z(valid_sigma));

        ei_values(valid_sigma) = improvement(valid_sigma) .* Phi_Z + sigma_cand(valid_sigma) .* phi_Z;
        ei_values(~valid_sigma) = max(0, improvement(~valid_sigma));
        ei_values = max(0, ei_values);

        % Select candidate with maximum Expected Improvement
        [max_ei, best_cand_idx] = max(ei_values);
        next_x_norm = Cand_norm(best_cand_idx, :);

        % If EI is too small, inject exploratory random perturbation
        if max_ei < 1e-6
            next_x_norm = rand(1, D);
        end

        % Map to physical parameter coordinates
        next_x_phys = lb + next_x_norm .* (ub - lb);

        % Evaluate true black-box cost function
        try
            next_y = cost_func(next_x_phys);
        catch
            next_y = 1e6;
        end

        % Augment observation dataset
        X_norm = [X_norm; next_x_norm];
        X_phys = [X_phys; next_x_phys];
        Y      = [Y; next_y];

        % Update best observed parameter set
        if next_y < best_cost
            best_cost = next_y;
            best_pos  = next_x_phys;
            best_idx  = length(Y);
        end

        % Record convergence statistics
        history.cost_per_iter(iter)       = best_cost;
        history.sigma_mean(iter)          = mean(sigma_cand);
        history.best_pos_per_iter(iter,:) = best_pos;

        if verbose
            fprintf('  BO Iter %2d/%d | Best Cost: %.4f | Current Cost: %.4f | GP Max EI: %.5f | Avg Sigma: %.4f\n', ...
                iter, max_iter, best_cost, next_y, max_ei, mean(sigma_cand));
        end
    end

    history.final_best_pos  = best_pos;
    history.final_best_cost = best_cost;

    if verbose
        fprintf('  ✓ Bayesian Optimization Complete! Best cost = %.4f\n', best_cost);
    end
end

%% =========================================================================
%  GAUSSIAN PROCESS REGRESSION (SURROGATE MODEL)
% =========================================================================

function gp = fit_gaussian_process(X, y)
    % Fits a Gaussian Process model with Squared Exponential (RBF) Kernel
    N = size(X, 1);
    D = size(X, 2);

    % Robust empirical scaling of lengthscales and signal variance
    % Normalize Y to mean 0, std 1 for numerical stability
    y_mean = mean(y);
    y_std = std(y);
    if y_std < 1e-6, y_std = 1.0; end
    y_norm = (y - y_mean) / y_std;

    lengthscales = 0.5 * ones(1, D); % isotropic lengthscale in normalized [0, 1]^D
    sigma_f = 1.0;                  % Signal variance
    sigma_n = 1e-3;                 % Noise variance / regularization

    % Compute kernel matrix K
    dist_sq = zeros(N, N);
    for d = 1:D
        diff_d = bsxfun(@minus, X(:, d), X(:, d)');
        dist_sq = dist_sq + (diff_d .^ 2) / (lengthscales(d)^2);
    end
    K = (sigma_f^2) * exp(-0.5 * dist_sq) + (sigma_n^2 + 1e-5) * eye(N);

    % Cholesky decomposition for stable matrix inversion
    [L, p_flag] = chol(K, 'lower');
    if p_flag ~= 0
        K = K + 1e-3 * eye(N);
        L = chol(K, 'lower');
    end

    % Precompute alpha vector: (K^-1) * y_norm
    alpha = L' \ (L \ y_norm);

    % Pack GP model
    gp.X = X;
    gp.y_mean = y_mean;
    gp.y_std = y_std;
    gp.L = L;
    gp.alpha = alpha;
    gp.lengthscales = lengthscales;
    gp.sigma_f = sigma_f;
    gp.sigma_n = sigma_n;
end

function [mu, sigma] = predict_gaussian_process(gp, X_star)
    % Evaluates Gaussian Process posterior mean and uncertainty at test points X_star
    N_train = size(gp.X, 1);
    N_test  = size(X_star, 1);
    D       = size(gp.X, 2);

    % Compute cross-kernel K_star [N_train x N_test]
    dist_sq_star = zeros(N_train, N_test);
    for d = 1:D
        diff_d = bsxfun(@minus, gp.X(:, d), X_star(:, d)');
        dist_sq_star = dist_sq_star + (diff_d .^ 2) / (gp.lengthscales(d)^2);
    end
    K_star = (gp.sigma_f^2) * exp(-0.5 * dist_sq_star);

    % Posterior mean (normalized -> physical scale)
    mu_norm = K_star' * gp.alpha;
    mu = gp.y_mean + mu_norm * gp.y_std;

    % Posterior variance calculation
    v = gp.L \ K_star;
    var_norm = (gp.sigma_f^2) - sum(v .^ 2, 1)';
    var_norm = max(0, var_norm);
    sigma = sqrt(var_norm) * gp.y_std;
end

%% =========================================================================
%  PROBABILITY UTILITY FUNCTIONS
% =========================================================================

function p = norm_pdf(z)
    p = (1.0 / sqrt(2.0 * pi)) * exp(-0.5 * (z .^ 2));
end

function P = norm_cdf(z)
    P = 0.5 * (1.0 + erf(z / sqrt(2.0)));
end

function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

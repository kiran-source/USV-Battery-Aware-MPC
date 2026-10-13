function [best_pos, best_cost, history] = PSO_optimizer(cost_func, lb, ub, options)
    % PSO_OPTIMIZER Particle Swarm Optimization for Controller Parameter Tuning
    %
    % A metaheuristic ML-based optimization algorithm that mimics the social
    % behaviour of bird flocking to find optimal controller parameters.
    %
    % Inputs:
    %   cost_func - Function handle @(params) -> scalar cost (lower = better)
    %   lb        - Lower bounds vector [1 x D]
    %   ub        - Upper bounds vector [1 x D]
    %   options   - Struct with PSO configuration:
    %               .n_particles  (default 25)
    %               .max_iter     (default 20)
    %               .w_inertia    (default 0.7)
    %               .c1_cognitive (default 1.5)
    %               .c2_social    (default 1.5)
    %               .verbose      (default true)
    %
    % Outputs:
    %   best_pos  - Best parameter vector found [1 x D]
    %   best_cost - Best (lowest) cost achieved
    %   history   - Struct with convergence data for plotting
    
    %% Default Options
    if nargin < 4, options = struct(); end
    n_particles = getfield_default(options, 'n_particles', 25);
    max_iter    = getfield_default(options, 'max_iter', 20);
    w           = getfield_default(options, 'w_inertia', 0.7);
    c1          = getfield_default(options, 'c1_cognitive', 1.5);
    c2          = getfield_default(options, 'c2_social', 1.5);
    verbose     = getfield_default(options, 'verbose', true);
    
    D = length(lb); % Dimension of search space
    
    %% 1. INITIALIZE SWARM
    % Random positions within bounds
    positions  = repmat(lb, n_particles, 1) + rand(n_particles, D) .* repmat(ub - lb, n_particles, 1);
    velocities = zeros(n_particles, D);
    
    % Evaluate initial fitness
    costs = inf(n_particles, 1);
    for i = 1:n_particles
        try
            costs(i) = cost_func(positions(i, :));
        catch
            costs(i) = 1e6; % Penalize failed evaluations
        end
    end
    
    % Personal bests
    p_best_pos  = positions;
    p_best_cost = costs;
    
    % Global best
    [g_best_cost, g_idx] = min(costs);
    g_best_pos = positions(g_idx, :);
    
    % History tracking
    history.cost_per_iter   = zeros(max_iter, 1);
    history.mean_cost       = zeros(max_iter, 1);
    history.best_pos_per_iter = zeros(max_iter, D);
    
    if verbose
        fprintf('  PSO Initialized: %d particles, %d dimensions, %d iterations\n', n_particles, D, max_iter);
        fprintf('  Initial best cost: %.4f\n', g_best_cost);
    end
    
    %% 2. PSO MAIN LOOP
    for iter = 1:max_iter
        % Linearly decreasing inertia weight (exploration -> exploitation)
        w_cur = w - (w - 0.4) * (iter / max_iter);
        
        for i = 1:n_particles
            % Random coefficients
            r1 = rand(1, D);
            r2 = rand(1, D);
            
            % Velocity update (cognitive + social + inertia)
            velocities(i, :) = w_cur * velocities(i, :) ...
                + c1 * r1 .* (p_best_pos(i, :) - positions(i, :)) ...
                + c2 * r2 .* (g_best_pos - positions(i, :));
            
            % Position update
            positions(i, :) = positions(i, :) + velocities(i, :);
            
            % Enforce bounds (reflection)
            for d = 1:D
                if positions(i, d) < lb(d)
                    positions(i, d) = lb(d);
                    velocities(i, d) = -0.5 * velocities(i, d);
                elseif positions(i, d) > ub(d)
                    positions(i, d) = ub(d);
                    velocities(i, d) = -0.5 * velocities(i, d);
                end
            end
            
            % Evaluate new position
            try
                new_cost = cost_func(positions(i, :));
            catch
                new_cost = 1e6;
            end
            costs(i) = new_cost;
            
            % Update personal best
            if new_cost < p_best_cost(i)
                p_best_cost(i) = new_cost;
                p_best_pos(i, :) = positions(i, :);
            end
            
            % Update global best
            if new_cost < g_best_cost
                g_best_cost = new_cost;
                g_best_pos = positions(i, :);
            end
        end
        
        % Record history
        history.cost_per_iter(iter)     = g_best_cost;
        history.mean_cost(iter)         = mean(costs);
        history.best_pos_per_iter(iter, :) = g_best_pos;
        
        if verbose
            fprintf('  Iter %2d/%d | Best Cost: %.4f | Mean Cost: %.4f | Inertia: %.3f\n', ...
                iter, max_iter, g_best_cost, mean(costs), w_cur);
        end
    end
    
    best_pos  = g_best_pos;
    best_cost = g_best_cost;
    history.final_best_pos  = best_pos;
    history.final_best_cost = best_cost;
    
    if verbose
        fprintf('  PSO Complete! Best cost = %.4f\n', best_cost);
    end
end

function val = getfield_default(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end

function map_data = build_map(p_mis)
    % BUILD_MAP Constructs 2D Navigation Map, Obstacle Grid, and Mission Graph Nodes
    
    if nargin < 1, p_mis = mission_parameters(); end
    
    res = p_mis.map.resolution;
    x_vec = p_mis.map.x_min:res:p_mis.map.x_max;
    y_vec = p_mis.map.y_min:res:p_mis.map.y_max;
    
    Nx = length(x_vec);
    Ny = length(y_vec);
    
    [X_grid, Y_grid] = meshgrid(x_vec, y_vec);
    grid_map = zeros(Ny, Nx); % 0 = Free water, 1 = Obstacle / Land
    
    %% 1. POPULATE OBSTACLES (Islands, reefs, restricted corridors)
    for obs_idx = 1:length(p_mis.obstacles)
        obs = p_mis.obstacles(obs_idx);
        if strcmp(obs.type, 'circle')
            cx = obs.center(1); cy = obs.center(2); r = obs.radius;
            dist_sq = (X_grid - cx).^2 + (Y_grid - cy).^2;
            grid_map(dist_sq <= (r + 15.0)^2) = 1; % Includes 15m safety inflation margin
        elseif strcmp(obs.type, 'polygon')
            in_poly = inpolygon(X_grid, Y_grid, obs.vertices(:,1), obs.vertices(:,2));
            grid_map(in_poly) = 1;
        end
    end
    
    map_data.x_vec = x_vec;
    map_data.y_vec = y_vec;
    map_data.X_grid = X_grid;
    map_data.Y_grid = Y_grid;
    map_data.grid_map = grid_map;
    map_data.resolution = res;
    map_data.nodes = p_mis.nodes;
    map_data.obstacles = p_mis.obstacles;
end

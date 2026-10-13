function animate_USV_mission(res, playback_speed, save_gif)
    % ANIMATE_USV_MISSION 2D Animated Hydrodynamic USV Marine Simulator
    %
    % Displays:
    %   - Moving USV boat hull polygon with true position (x, y) and heading (psi)
    %   - Ocean wave field & water current velocity vector flow field
    %   - Obstacles, Islands, Shallow Reefs, and Marine Corridors
    %   - Base, Waypoints, Charging Dock Station, and Destination
    %   - Planned A* path vs dynamic wake trail
    %   - Real-time telemetry HUD (Speed, Heading, Battery SOC Gauge, Power, Mode)
    %
    % Usage:
    %   animate_USV_mission()            % Runs default Battery-Aware MPC scenario
    %   animate_USV_mission(res, 5, true)% 5x speed, saves animated GIF
    
    if nargin < 1 || isempty(res)
        % Load default Battery-Aware MPC simulation
        if exist('simulation_results.mat', 'file')
            data = load('simulation_results.mat');
            if isfield(data, 'all_results')
                res = data.all_results{3}; % Exp 3: Battery-Aware MPC
            else
                p = load_all_parameters();
                res = run_simulation(p);
            end
        else
            p = load_all_parameters();
            res = run_simulation(p);
        end
    end
    if nargin < 2 || isempty(playback_speed), playback_speed = 8.0; end
    if nargin < 3, save_gif = false; end
    
    %% 1. INITIALIZE FIGURE & CANVAS
    fig = figure('Name', 'USV Marine Hydrodynamic Simulator', ...
        'Position', [50, 50, 1200, 750], 'Color', [0.05 0.08 0.14]);
    ax = axes('Parent', fig, 'Color', [0.08 0.12 0.20], 'Position', [0.05 0.08 0.65 0.85]);
    axis equal; grid on; hold on;
    set(ax, 'XColor', [0.4 0.6 0.8], 'YColor', [0.4 0.6 0.8], 'GridColor', [0.2 0.3 0.45]);
    
    xlim([-100, 1400]); ylim([-200, 850]);
    xlabel('East X [meters]', 'Color', 'w', 'FontSize', 11);
    ylabel('North Y [meters]', 'Color', 'w', 'FontSize', 11);
    title('2D HYDRODYNAMIC MARINE SIMULATOR (BATTERY-AWARE MPC)', 'Color', 'w', 'FontSize', 13, 'FontWeight', 'bold');
    
    %% 2. RENDER STATIC ENVIRONMENT (Obstacles, Islands, Waypoints)
    map_data = res.map_data;
    
    % Ocean Current Vector Field (Arrows)
    [X_grid, Y_grid] = meshgrid(0:150:1300, -100:150:750);
    Vc_speed = 0.35; beta_c = pi; % Current flowing West
    U_curr = Vc_speed * cos(beta_c) * ones(size(X_grid));
    V_curr = Vc_speed * sin(beta_c) * ones(size(X_grid));
    quiver(ax, X_grid, Y_grid, U_curr, V_curr, 0.4, 'Color', [0.15 0.35 0.55], 'LineWidth', 1.0, 'DisplayName', 'Ocean Current (0.35 m/s W)');
    
    % Render Islands and Obstacles
    for k = 1:length(map_data.obstacles)
        obs = map_data.obstacles(k);
        if strcmp(obs.type, 'circle')
            theta_c = linspace(0, 2*pi, 100);
            plot(ax, obs.center(1) + obs.radius*cos(theta_c), obs.center(2) + obs.radius*sin(theta_c), '-', 'Color', [0.9 0.4 0.2], 'LineWidth', 1.5, 'HandleVisibility', 'off');
            fill(ax, obs.center(1) + obs.radius*cos(linspace(0,2*pi,30)), ...
                     obs.center(2) + obs.radius*sin(linspace(0,2*pi,30)), ...
                     [0.45 0.30 0.20], 'EdgeColor', [0.8 0.4 0.2]);
        elseif strcmp(obs.type, 'polygon')
            fill(ax, obs.vertices(:,1), obs.vertices(:,2), [0.45 0.30 0.20], ...
                'FaceAlpha', 0.8, 'EdgeColor', [0.8 0.4 0.2], 'LineWidth', 1.5);
        end
    end
    
    % Plot Mission Nodes
    nodes = map_data.nodes;
    plot(ax, nodes.base.x, nodes.base.y, 'gp', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Base Dock');
    plot(ax, nodes.dest.x, nodes.dest.y, 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'r', 'DisplayName', 'Final Destination');
    plot(ax, nodes.dock.x, nodes.dock.y, 'ms', 'MarkerSize', 12, 'MarkerFaceColor', 'm', 'DisplayName', 'Charging Dock');
    plot(ax, nodes.harbor.x, nodes.harbor.y, 'co', 'MarkerSize', 11, 'MarkerFaceColor', 'c', 'DisplayName', 'Safe Harbor');
    
    % Planned Route Path
    plot(ax, res.waypoints(:,1), res.waypoints(:,2), 'c*--', 'LineWidth', 1.4, 'DisplayName', 'Planned A* Route');
    
    %% 3. CREATE DYNAMIC GRAPHICAL OBJECTS
    % Wake Trail
    h_trail = plot(ax, NaN, NaN, 'w-', 'LineWidth', 1.2, 'DisplayName', 'USV Wake Trail');
    
    % Boat Hull Polygon Template (Normalized boat geometry)
    L_hull = 25.0; % Rendered visual hull length [m]
    W_hull = 10.0; % Rendered visual hull width [m]
    hull_x_tmpl = [ L_hull/2,  L_hull/4, -L_hull/2, -L_hull/2,  L_hull/4,  L_hull/2 ];
    hull_y_tmpl = [ 0,         W_hull/2,  W_hull/2, -W_hull/2, -W_hull/2,  0        ];
    
    % Boat Hull Patch
    h_hull = patch('Parent', ax, 'XData', [], 'YData', [], 'FaceColor', [0.2 0.8 0.9], ...
        'EdgeColor', 'w', 'LineWidth', 1.5);
    
    % Dual Thruster Wake Lines
    h_thrust_L = plot(ax, NaN, NaN, 'r-', 'LineWidth', 2.0);
    h_thrust_R = plot(ax, NaN, NaN, 'r-', 'LineWidth', 2.0);
    
    % Wave Crest Animations (Dynamic wave crests moving across water)
    num_waves = 12;
    wave_x = linspace(-50, 1350, num_waves);
    wave_lines = gobjects(num_waves, 1);
    for w = 1:num_waves
        wave_lines(w) = plot(ax, [wave_x(w), wave_x(w)+60], [-150, -150], ':', ...
            'Color', [0.2 0.45 0.70], 'LineWidth', 1.2);
    end
    
    legend(ax, 'Location', 'northwest', 'TextColor', 'w', 'Color', [0.1 0.15 0.22]);
    
    %% 4. TELEMETRY HUD PANEL (On the Right)
    hud_panel = uipanel('Parent', fig, 'Position', [0.72, 0.08, 0.25, 0.85], ...
        'BackgroundColor', [0.10 0.13 0.18], 'ForegroundColor', 'w', ...
        'Title', 'LIVE VESSEL TELEMETRY', 'FontSize', 11, 'FontWeight', 'bold');
    
    h_time_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.91, 0.90, 0.06], 'String', 'TIME: 0.0 s', ...
        'FontSize', 12, 'FontWeight', 'bold', 'ForegroundColor', 'c', 'BackgroundColor', [0.15 0.18 0.25]);
    
    h_speed_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.82, 0.90, 0.06], 'String', 'SURGE SPEED: 0.00 m/s', ...
        'FontSize', 11, 'ForegroundColor', 'w', 'BackgroundColor', [0.12 0.15 0.20]);
    
    h_heading_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.74, 0.90, 0.06], 'String', 'HEADING: 0.0 deg', ...
        'FontSize', 11, 'ForegroundColor', 'w', 'BackgroundColor', [0.12 0.15 0.20]);
    
    h_soc_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.65, 0.90, 0.06], 'String', 'BATTERY: 80.0% SOC', ...
        'FontSize', 12, 'FontWeight', 'bold', 'ForegroundColor', [0.2 0.9 0.3], 'BackgroundColor', [0.15 0.22 0.20]);
    
    h_power_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.56, 0.90, 0.06], 'String', 'POWER: 0.0 Watts', ...
        'FontSize', 11, 'ForegroundColor', 'y', 'BackgroundColor', [0.12 0.15 0.20]);
    
    h_energy_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.47, 0.90, 0.06], 'String', 'ENERGY: 0.0 Wh', ...
        'FontSize', 11, 'ForegroundColor', 'w', 'BackgroundColor', [0.12 0.15 0.20]);
    
    h_thrust_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.38, 0.90, 0.06], 'String', 'THRUST (T): 0.0 N', ...
        'FontSize', 11, 'ForegroundColor', 'w', 'BackgroundColor', [0.12 0.15 0.20]);
    
    h_range_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.29, 0.90, 0.06], 'String', 'EST RANGE: 0.0 m', ...
        'FontSize', 11, 'ForegroundColor', [0.9 0.6 0.9], 'BackgroundColor', [0.12 0.15 0.20]);
    
    h_mode_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.18, 0.90, 0.08], 'String', 'MODE: CRUISE', ...
        'FontSize', 12, 'FontWeight', 'bold', 'ForegroundColor', 'g', 'BackgroundColor', [0.10 0.25 0.15]);
    
    h_feas_txt = uicontrol('Parent', hud_panel, 'Style', 'text', 'Units', 'normalized', ...
        'Position', [0.05, 0.06, 0.90, 0.08], 'String', 'STATUS: FEASIBLE', ...
        'FontSize', 12, 'FontWeight', 'bold', 'ForegroundColor', [0.2 0.9 0.4], 'BackgroundColor', [0.10 0.25 0.15]);
    
    %% 5. ANIMATION PLAYBACK LOOP
    dt_sim = res.t(2) - res.t(1);
    step_skip = max(round(playback_speed * 0.03 / dt_sim), 1);
    N_total = length(res.t);
    
    mode_names = {'1: CRUISE', '2: ECONOMY', '3: CHARGING DOCK', '4: RETURN TO BASE', '5: EMERGENCY'};
    mode_colors = {[0.2 0.8 0.3], [0.9 0.8 0.2], [0.8 0.3 0.8], [0.9 0.5 0.1], [0.9 0.2 0.2]};
    
    for i = 1:step_skip:N_total
        if ~ishandle(fig), break; end
        
        t_cur   = res.t(i);
        x_cur   = res.x(i);
        y_cur   = res.y(i);
        psi_cur = res.psi(i);
        u_cur   = res.u(i);
        soc_cur = res.SOC(i);
        P_cur   = res.P(i);
        T_cur   = res.T(i);
        mode_id = max(min(res.mode(i), 5), 1);
        
        % 1. Update USV Hull Geometry (Rotation + Translation)
        R_mat = [cos(psi_cur), -sin(psi_cur); sin(psi_cur), cos(psi_cur)];
        rot_pts = R_mat * [hull_x_tmpl; hull_y_tmpl];
        set(h_hull, 'XData', rot_pts(1,:) + x_cur, 'YData', rot_pts(2,:) + y_cur);
        
        % 2. Update Wake Trail
        set(h_trail, 'XData', res.x(1:i), 'YData', res.y(1:i));
        
        % 3. Update Dual Thruster Wake Plumes
        stern_L = R_mat * [-L_hull/2;  W_hull/3] + [x_cur; y_cur];
        stern_R = R_mat * [-L_hull/2; -W_hull/3] + [x_cur; y_cur];
        plume_len = min(T_cur / 10.0, 20.0);
        plume_L = stern_L - R_mat * [plume_len; 0];
        plume_R = stern_R - R_mat * [plume_len; 0];
        set(h_thrust_L, 'XData', [stern_L(1), plume_L(1)], 'YData', [stern_L(2), plume_L(2)]);
        set(h_thrust_R, 'XData', [stern_R(1), plume_R(1)], 'YData', [stern_R(2), plume_R(2)]);
        
        % 4. Animate Ambient Waves (Propagating wave fronts)
        for w = 1:num_waves
            w_y = mod(-100 + w*75 + t_cur*8.0, 950) - 100;
            set(wave_lines(w), 'YData', [w_y, w_y + 40]);
        end
        
        % 5. Update Telemetry HUD Texts
        set(h_time_txt,    'String', sprintf('TIME: %.1f s (%.1fx)', t_cur, playback_speed));
        set(h_speed_txt,   'String', sprintf('SURGE SPEED: %.2f m/s', u_cur));
        set(h_heading_txt, 'String', sprintf('HEADING: %.1f deg', rad2deg(psi_cur)));
        set(h_soc_txt,     'String', sprintf('BATTERY: %.1f%% SOC', soc_cur));
        set(h_power_txt,   'String', sprintf('POWER: %.1f Watts', P_cur));
        set(h_energy_txt,  'String', sprintf('ENERGY: %.1f Wh', res.E_used_Wh(i)));
        set(h_thrust_txt,  'String', sprintf('THRUST: %.1f N', T_cur));
        set(h_range_txt,   'String', sprintf('EST RANGE: %.0f m', res.R_est(i)));
        
        % Dynamic Battery Color
        if soc_cur > 50.0
            set(h_soc_txt, 'ForegroundColor', [0.2 0.9 0.3]);
        elseif soc_cur > 25.0
            set(h_soc_txt, 'ForegroundColor', [0.9 0.8 0.2]);
        else
            set(h_soc_txt, 'ForegroundColor', [0.9 0.2 0.2]);
        end
        
        % Mode Badge Color
        set(h_mode_txt, 'String', sprintf('MODE: %s', mode_names{mode_id}), ...
            'ForegroundColor', mode_colors{mode_id});
        
        % Feasibility Status
        if res.is_feasible(i)
            set(h_feas_txt, 'String', 'STATUS: FEASIBLE', 'ForegroundColor', [0.2 0.9 0.4], 'BackgroundColor', [0.1 0.25 0.15]);
        else
            set(h_feas_txt, 'String', 'STATUS: AT RISK', 'ForegroundColor', [0.9 0.2 0.2], 'BackgroundColor', [0.3 0.1 0.1]);
        end
        
        drawnow limitrate;
    end
end

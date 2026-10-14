function fig = create_dashboard(res, output_dir)
    % CREATE_DASHBOARD Generates Real-Time USV Mission Telemetry Dashboard with Crisp White Background
    
    if nargin < 2, output_dir = pwd; end
    
    % Force clean white theme defaults
    set(0, 'DefaultFigureColor', 'w');
    set(0, 'DefaultAxesColor', 'w');
    set(0, 'DefaultAxesXColor', [0.15 0.15 0.15]);
    set(0, 'DefaultAxesYColor', [0.15 0.15 0.15]);
    set(0, 'DefaultTextColor', [0.1 0.1 0.1]);
    
    fig = figure('Name', 'USV Battery-Aware MPC Mission Dashboard', ...
        'Position', [50 50 1400 850], 'Color', 'w', 'Visible', 'off');
    
    % Top Header Card
    annotation(fig, 'textbox', [0.03, 0.93, 0.94, 0.05], ...
        'String', sprintf('BATTERY-AWARE 3-DOF USV MPC MISSION DASHBOARD  |  SCENARIO: %s', res.name), ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', [0.08 0.18 0.35], ...
        'BackgroundColor', [0.93 0.96 0.99], 'EdgeColor', [0.72 0.82 0.92], 'LineWidth', 1.2, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    
    % Telemetry Summary Cards
    card_str1 = sprintf('TRAVEL TIME\n%.1f s (%.1f min)', res.summary.travel_time_s, res.summary.travel_time_s/60);
    card_str2 = sprintf('TOTAL ENERGY\n%.1f Wh (%.2f MJ)', res.summary.energy_Wh, res.summary.energy_Wh*3600/1e6);
    card_str3 = sprintf('FINAL BATTERY\n%.1f%% SOC', res.summary.final_soc);
    card_str4 = sprintf('AVG POWER\n%.1f Watts', res.summary.avg_power_W);
    card_str5 = sprintf('CROSS-TRACK RMSE\n%.2f meters', res.summary.rmse_cte);
    
    card_bgs   = {[0.92 0.95 0.99], [0.99 0.94 0.91], [0.92 0.97 0.93], [0.96 0.93 0.98], [0.92 0.96 0.96]};
    card_edges = {[0.70 0.82 0.95], [0.95 0.75 0.65], [0.70 0.90 0.75], [0.85 0.75 0.92], [0.72 0.88 0.88]};
    card_texts = {[0.10 0.30 0.65], [0.75 0.25 0.10], [0.10 0.55 0.20], [0.45 0.15 0.60], [0.10 0.45 0.45]};
    cards = {card_str1, card_str2, card_str3, card_str4, card_str5};
    
    for c = 1:5
        x_pos = 0.03 + (c-1) * 0.19;
        annotation(fig, 'textbox', [x_pos, 0.84, 0.17, 0.07], ...
            'String', cards{c}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', card_texts{c}, ...
            'BackgroundColor', card_bgs{c}, 'EdgeColor', card_edges{c}, 'LineWidth', 1.2, ...
            'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    end
    
    % Subplot 1: 2D Trajectory & Obstacle Map
    subplot('Position', [0.05, 0.45, 0.42, 0.35]);
    for k = 1:length(res.map_data.obstacles)
        obs = res.map_data.obstacles(k);
        if strcmp(obs.type, 'circle')
            theta_c = linspace(0, 2*pi, 100);
            cx = obs.center(1) + obs.radius * cos(theta_c);
            cy = obs.center(2) + obs.radius * sin(theta_c);
            plot(cx, cy, '-', 'Color', [0.65 0.35 0.15], 'LineWidth', 1.5, 'HandleVisibility', 'off'); hold on;
            fill(cx, cy, [0.88 0.72 0.58], 'FaceAlpha', 0.5, 'EdgeColor', [0.65 0.35 0.15], 'LineWidth', 1.5, 'HandleVisibility', 'off');
        elseif strcmp(obs.type, 'polygon')
            fill(obs.vertices(:,1), obs.vertices(:,2), [0.88 0.72 0.58], 'FaceAlpha', 0.6, 'EdgeColor', [0.65 0.35 0.15], 'LineWidth', 1.5); hold on;
        end
    end
    hold on;
    plot(res.waypoints(:,1), res.waypoints(:,2), 'k*--', 'LineWidth', 1.2, 'DisplayName', 'Planned Path');
    plot(res.x, res.y, '-', 'Color', [0.05 0.45 0.85], 'LineWidth', 2.2, 'DisplayName', 'USV Trajectory');
    nodes = res.map_data.nodes;
    plot(nodes.base.x, nodes.base.y, 'gp', 'MarkerSize', 10, 'MarkerFaceColor', [0.10 0.70 0.20], 'DisplayName', 'Base');
    plot(nodes.dest.x, nodes.dest.y, 'rp', 'MarkerSize', 10, 'MarkerFaceColor', [0.85 0.15 0.15], 'DisplayName', 'Destination');
    plot(nodes.dock.x, nodes.dock.y, 'ms', 'MarkerSize', 8,  'MarkerFaceColor', [0.75 0.15 0.75], 'DisplayName', 'Dock');
    title('2D Mission Navigation & Obstacles', 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold');
    xlabel('East X [m]', 'Color', [0.15 0.15 0.15]); ylabel('North Y [m]', 'Color', [0.15 0.15 0.15]);
    set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);
    legend('Location', 'northwest', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);
    grid on; axis equal;
    
    % Subplot 2: Speed Tracking
    subplot('Position', [0.53, 0.45, 0.42, 0.35]);
    plot(res.t, res.u_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target u_{ref}'); hold on;
    plot(res.t, res.u,     '-',   'Color', [0.05 0.45 0.85], 'LineWidth', 2.0, 'DisplayName', 'Surge Speed u');
    plot(res.t, res.v,     ':',   'Color', [0.70 0.15 0.70], 'LineWidth', 1.4, 'DisplayName', 'Sway Speed v');
    title('Velocity Tracking [m/s]', 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold');
    xlabel('Time [s]', 'Color', [0.15 0.15 0.15]); ylabel('Speed [m/s]', 'Color', [0.15 0.15 0.15]);
    set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);
    legend('Location', 'best', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]); grid on;
    
    % Subplot 3: Battery SOC & Achievable Range
    subplot('Position', [0.05, 0.06, 0.42, 0.32]);
    yyaxis left;
    plot(res.t, res.SOC, '-', 'Color', [0.05 0.65 0.25], 'LineWidth', 2.2);
    ylabel('Battery SOC [%]', 'Color', [0.05 0.65 0.25]);
    set(gca, 'YColor', [0.05 0.65 0.25]);
    ylim([0 100]);
    yyaxis right;
    plot(res.t, res.R_est, '--', 'Color', [0.65 0.15 0.65], 'LineWidth', 1.6);
    ylabel('Est Range [m]', 'Color', [0.65 0.15 0.65]);
    set(gca, 'YColor', [0.65 0.15 0.65]);
    title('Battery State of Charge & Estimated Range', 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold');
    xlabel('Time [s]', 'Color', [0.15 0.15 0.15]);
    set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);
    grid on;
    
    % Subplot 4: Power Draw & Commanded Thrust
    subplot('Position', [0.53, 0.06, 0.42, 0.32]);
    yyaxis left;
    plot(res.t, res.P, '-', 'Color', [0.85 0.35 0.0], 'LineWidth', 1.8);
    ylabel('Electrical Power [W]', 'Color', [0.85 0.35 0.0]);
    set(gca, 'YColor', [0.85 0.35 0.0]);
    yyaxis right;
    plot(res.t, res.T, '--', 'Color', [0.15 0.35 0.65], 'LineWidth', 1.6);
    ylabel('Thrust [N]', 'Color', [0.15 0.35 0.65]);
    set(gca, 'YColor', [0.15 0.35 0.65]);
    title('Instantaneous Power & Thruster Force', 'Color', [0.1 0.1 0.1], 'FontWeight', 'bold');
    xlabel('Time [s]', 'Color', [0.15 0.15 0.15]);
    set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);
    grid on;
    
    saveas(fig, fullfile(output_dir, 'dashboard.png'));
end

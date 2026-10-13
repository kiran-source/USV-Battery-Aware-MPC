function fig = create_dashboard(res, output_dir)
    % CREATE_DASHBOARD Generates Real-Time USV Mission Telemetry Dashboard
    
    if nargin < 2, output_dir = pwd; end
    
    fig = figure('Name', 'USV Battery-Aware MPC Mission Dashboard', ...
        'Position', [50 50 1400 850], 'Color', [0.12 0.14 0.18], 'Visible', 'off');
    
    % Header Card
    annotation(fig, 'textbox', [0.03, 0.93, 0.94, 0.05], ...
        'String', sprintf('BATTERY-AWARE 3-DOF USV MPC MISSION DASHBOARD  |  SCENARIO: %s', res.name), ...
        'FontSize', 14, 'FontWeight', 'bold', 'Color', 'w', ...
        'BackgroundColor', [0.18 0.22 0.28], 'EdgeColor', [0.3 0.4 0.5], ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle');
    
    % Telemetry Summary Cards
    card_str1 = sprintf('TRAVEL TIME\n%.1f s (%.1f min)', res.summary.travel_time_s, res.summary.travel_time_s/60);
    card_str2 = sprintf('TOTAL ENERGY\n%.1f Wh (%.2f MJ)', res.summary.energy_Wh, res.summary.energy_Wh*3600/1e6);
    card_str3 = sprintf('FINAL BATTERY\n%.1f%% SOC', res.summary.final_soc);
    card_str4 = sprintf('AVG POWER\n%.1f Watts', res.summary.avg_power_W);
    card_str5 = sprintf('CROSS-TRACK RMSE\n%.2f meters', res.summary.rmse_cte);
    
    card_colors = {[0.2 0.4 0.6], [0.5 0.3 0.2], [0.2 0.5 0.3], [0.4 0.3 0.5], [0.3 0.5 0.5]};
    cards = {card_str1, card_str2, card_str3, card_str4, card_str5};
    
    for c = 1:5
        x_pos = 0.03 + (c-1) * 0.19;
        annotation(fig, 'textbox', [x_pos, 0.84, 0.17, 0.07], ...
            'String', cards{c}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'w', ...
            'BackgroundColor', card_colors{c}, 'EdgeColor', [0.5 0.6 0.7], ...
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
            plot(cx, cy, '-', 'Color', [0.9 0.4 0.2], 'LineWidth', 1.5, 'HandleVisibility', 'off');
            fill(cx, cy, [0.8 0.5 0.3], 'FaceAlpha', 0.4, 'EdgeColor', [0.9 0.4 0.2], 'LineWidth', 1.5, 'HandleVisibility', 'off');
        elseif strcmp(obs.type, 'polygon')
            fill(obs.vertices(:,1), obs.vertices(:,2), [0.8 0.5 0.3], 'FaceAlpha', 0.6, 'EdgeColor', [0.9 0.4 0.2]);
        end
    end
    hold on;
    plot(res.waypoints(:,1), res.waypoints(:,2), 'c*--', 'LineWidth', 1.2, 'DisplayName', 'Planned Path');
    plot(res.x, res.y, 'y-', 'LineWidth', 2.2, 'DisplayName', 'USV Trajectory');
    nodes = res.map_data.nodes;
    plot(nodes.base.x, nodes.base.y, 'gp', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(nodes.dest.x, nodes.dest.y, 'rp', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    plot(nodes.dock.x, nodes.dock.y, 'ms', 'MarkerSize', 8, 'MarkerFaceColor', 'm');
    title('2D Mission Navigation & Obstacles', 'Color', 'w');
    xlabel('East X [m]', 'Color', 'w'); ylabel('North Y [m]', 'Color', 'w');
    set(gca, 'Color', [0.08 0.10 0.14], 'XColor', 'w', 'YColor', 'w');
    grid on; axis equal;
    
    % Subplot 2: Speed Tracking
    subplot('Position', [0.53, 0.45, 0.42, 0.35]);
    plot(res.t, res.u_ref, 'r--', 'LineWidth', 1.5, 'DisplayName', 'u_{ref}'); hold on;
    plot(res.t, res.u,     'c-',  'LineWidth', 2.0, 'DisplayName', 'Surge u');
    plot(res.t, res.v,     'm:',  'LineWidth', 1.2, 'DisplayName', 'Sway v');
    title('Velocity Tracking [m/s]', 'Color', 'w');
    xlabel('Time [s]', 'Color', 'w'); ylabel('Speed [m/s]', 'Color', 'w');
    set(gca, 'Color', [0.08 0.10 0.14], 'XColor', 'w', 'YColor', 'w');
    legend('Location', 'best'); grid on;
    
    % Subplot 3: Battery SOC & Achievable Range
    subplot('Position', [0.05, 0.06, 0.42, 0.32]);
    yyaxis left;
    plot(res.t, res.SOC, 'g-', 'LineWidth', 2.2);
    ylabel('Battery SOC [%]', 'Color', 'g');
    set(gca, 'YColor', 'g');
    ylim([0 100]);
    yyaxis right;
    plot(res.t, res.R_est, 'm--', 'LineWidth', 1.6);
    ylabel('Est Range [m]', 'Color', 'm');
    set(gca, 'YColor', 'm');
    title('Battery State of Charge & Estimated Range', 'Color', 'w');
    xlabel('Time [s]', 'Color', 'w');
    set(gca, 'Color', [0.08 0.10 0.14], 'XColor', 'w');
    grid on;
    
    % Subplot 4: Power Draw & Commanded Thrust
    subplot('Position', [0.53, 0.06, 0.42, 0.32]);
    yyaxis left;
    plot(res.t, res.P, 'y-', 'LineWidth', 1.8);
    ylabel('Electrical Power [W]', 'Color', 'y');
    set(gca, 'YColor', 'y');
    yyaxis right;
    plot(res.t, res.T, 'w--', 'LineWidth', 1.4);
    ylabel('Thrust [N]', 'Color', 'w');
    set(gca, 'YColor', 'w');
    title('Instantaneous Power & Thruster Force', 'Color', 'w');
    xlabel('Time [s]', 'Color', 'w');
    set(gca, 'Color', [0.08 0.10 0.14], 'XColor', 'w');
    grid on;
    
    saveas(fig, fullfile(output_dir, 'dashboard.png'));
end

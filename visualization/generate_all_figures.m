function generate_all_figures(all_results, output_dir)
    % GENERATE_ALL_FIGURES Generates & Exports All 15 Required Benchmark Figures
    
    if nargin < 2, output_dir = pwd; end
    
    set(0, 'DefaultAxesFontSize', 10);
    set(0, 'DefaultAxesFontName', 'Arial');
    
    res1 = all_results{1}; % PID
    res2 = all_results{2}; % Conv MPC
    res3 = all_results{3}; % Bat MPC 80%
    res4 = all_results{4}; % Bat MPC 60%
    res5 = all_results{5}; % Bat MPC 40%
    res6 = all_results{6}; % Bat MPC 20%
    res7 = all_results{7}; % Opposing Current
    res8 = all_results{8}; % Cross Current
    res9 = all_results{9}; % Wind
    res10 = all_results{10}; % Full Env
    res11 = all_results{11}; % Shortest Route
    
    %% FIGURE 1: 2D USV Trajectory & Obstacle Map
    fig1 = figure('Name', 'Fig 1: 2D Trajectory', 'Position', [100 100 800 600], 'Color', 'w', 'Visible', 'off');
    plot_map_obstacles(res3.map_data); hold on;
    plot(res1.x, res1.y, 'r--', 'LineWidth', 1.8, 'DisplayName', 'PID Path');
    plot(res2.x, res2.y, 'b-.', 'LineWidth', 1.8, 'DisplayName', 'Conventional MPC');
    plot(res3.x, res3.y, 'g-',  'LineWidth', 2.2, 'DisplayName', 'Battery-Aware MPC');
    plot(res3.waypoints(:,1), res3.waypoints(:,2), 'k*--', 'LineWidth', 1.2, 'DisplayName', 'Planned Waypoints');
    xlabel('Global East X [m]'); ylabel('Global North Y [m]');
    title('FIGURE 1: 2D USV Navigation Trajectory & Obstacle Avoidance');
    legend('Location', 'northwest'); grid on; axis equal;
    saveas(fig1, fullfile(output_dir, 'fig1_2D_trajectory.png'));
    
    %% FIGURE 2: Planned Path vs Actual Track
    fig2 = figure('Name', 'Fig 2: Path Tracking', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res3.x, res3.y, 'b-', 'LineWidth', 2.0, 'DisplayName', 'Actual USV Track'); hold on;
    plot(res3.waypoints(:,1), res3.waypoints(:,2), 'ro--', 'LineWidth', 1.5, 'MarkerSize', 6, 'DisplayName', 'A* Waypoint Path');
    xlabel('Global X [m]'); ylabel('Global Y [m]');
    title('FIGURE 2: Planned Line-of-Sight Path vs Actual Track');
    legend('Location', 'best'); grid on; axis equal;
    saveas(fig2, fullfile(output_dir, 'fig2_planned_vs_actual_path.png'));
    
    %% FIGURE 3: Speed Tracking (Surge & Sway)
    fig3 = figure('Name', 'Fig 3: Speed Tracking', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res3.t, res3.u_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target Speed u_{ref}'); hold on;
    plot(res3.t, res3.u,     'b-',  'LineWidth', 1.8, 'DisplayName', 'Surge Speed u');
    plot(res3.t, res3.v,     'm:',  'LineWidth', 1.4, 'DisplayName', 'Sway Speed v');
    xlabel('Time [s]'); ylabel('Velocity [m/s]');
    title('FIGURE 3: USV Velocity Profiles (Surge & Sway)');
    legend('Location', 'best'); grid on;
    saveas(fig3, fullfile(output_dir, 'fig3_speed_vs_time.png'));
    
    %% FIGURE 4: Desired vs Actual Speed (PID vs MPC)
    fig4 = figure('Name', 'Fig 4: Speed Comparison', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res1.t, res1.u, 'r--', 'LineWidth', 1.6, 'DisplayName', 'PID Speed'); hold on;
    plot(res2.t, res2.u, 'b-.', 'LineWidth', 1.6, 'DisplayName', 'Conventional MPC Speed');
    plot(res3.t, res3.u, 'g-',  'LineWidth', 2.0, 'DisplayName', 'Battery-Aware MPC Speed');
    plot(res3.t, res3.u_ref, 'k:', 'LineWidth', 1.5, 'DisplayName', 'Target Reference u_{ref}');
    xlabel('Time [s]'); ylabel('Speed [m/s]');
    title('FIGURE 4: Controller Speed Comparison (PID vs Conventional MPC vs Battery-Aware MPC)');
    legend('Location', 'best'); grid on;
    saveas(fig4, fullfile(output_dir, 'fig4_desired_vs_actual_speed.png'));
    
    %% FIGURE 5: Heading & Yaw Rate
    fig5 = figure('Name', 'Fig 5: Heading Tracking', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    subplot(2,1,1);
    plot(res3.t, rad2deg(res3.psi_ref), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Target \psi_{ref}'); hold on;
    plot(res3.t, rad2deg(res3.psi),     'b-',  'LineWidth', 1.8, 'DisplayName', 'Actual Heading \psi');
    ylabel('Heading [deg]'); title('FIGURE 5: USV Heading & Yaw Rate Response');
    legend('Location', 'best'); grid on;
    
    subplot(2,1,2);
    plot(res3.t, rad2deg(res3.r), 'm-', 'LineWidth', 1.6);
    xlabel('Time [s]'); ylabel('Yaw Rate r [deg/s]'); grid on;
    saveas(fig5, fullfile(output_dir, 'fig5_heading_vs_time.png'));
    
    %% FIGURE 6: Cross-Track Error
    fig6 = figure('Name', 'Fig 6: Cross-Track Error', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res1.t, res1.e_y, 'r--', 'LineWidth', 1.5, 'DisplayName', 'PID CTE'); hold on;
    plot(res2.t, res2.e_y, 'b-.', 'LineWidth', 1.5, 'DisplayName', 'Conv MPC CTE');
    plot(res3.t, res3.e_y, 'g-',  'LineWidth', 1.8, 'DisplayName', 'Battery-Aware MPC CTE');
    yline(0, 'k:');
    xlabel('Time [s]'); ylabel('Cross-Track Error e_y [m]');
    title('FIGURE 6: Cross-Track Error Comparison (Path Tracking Accuracy)');
    legend('Location', 'best'); grid on;
    saveas(fig6, fullfile(output_dir, 'fig6_cross_track_error.png'));
    
    %% FIGURE 7: Battery SOC vs Time
    fig7 = figure('Name', 'Fig 7: Battery SOC', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res1.t, res1.SOC, 'r--', 'LineWidth', 1.8, 'DisplayName', 'PID (80% Start)'); hold on;
    plot(res2.t, res2.SOC, 'b-.', 'LineWidth', 1.8, 'DisplayName', 'Conv MPC (80% Start)');
    plot(res3.t, res3.SOC, 'g-',  'LineWidth', 2.2, 'DisplayName', 'Battery-Aware MPC (80% Start)');
    yline(20, 'k:', 'Reserve Threshold (20%)', 'LineWidth', 1.2);
    xlabel('Time [s]'); ylabel('Battery SOC [%]');
    title('FIGURE 7: Battery State of Charge (SOC %) Depletion Curves');
    legend('Location', 'southwest'); grid on; ylim([0 100]);
    saveas(fig7, fullfile(output_dir, 'fig7_battery_soc_vs_time.png'));
    
    %% FIGURE 8: Instantaneous Power Draw
    fig8 = figure('Name', 'Fig 8: Power Draw', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res1.t, res1.P, 'r--', 'LineWidth', 1.4, 'DisplayName', 'PID Power'); hold on;
    plot(res2.t, res2.P, 'b-.', 'LineWidth', 1.4, 'DisplayName', 'Conv MPC Power');
    plot(res3.t, res3.P, 'g-',  'LineWidth', 1.8, 'DisplayName', 'Battery-Aware MPC Power');
    xlabel('Time [s]'); ylabel('Total Electrical Power [W]');
    title('FIGURE 8: Instantaneous Battery Power Draw (P_{prop} + P_{hotel})');
    legend('Location', 'best'); grid on;
    saveas(fig8, fullfile(output_dir, 'fig8_power_consumption_vs_time.png'));
    
    %% FIGURE 9: Cumulative Energy vs Distance
    fig9 = figure('Name', 'Fig 9: Cumulative Energy', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    d1 = [0; cumsum(sqrt(diff(res1.x).^2 + diff(res1.y).^2))];
    d2 = [0; cumsum(sqrt(diff(res2.x).^2 + diff(res2.y).^2))];
    d3 = [0; cumsum(sqrt(diff(res3.x).^2 + diff(res3.y).^2))];
    plot(d1, res1.E_used_Wh, 'r--', 'LineWidth', 1.8, 'DisplayName', 'PID'); hold on;
    plot(d2, res2.E_used_Wh, 'b-.', 'LineWidth', 1.8, 'DisplayName', 'Conv MPC');
    plot(d3, res3.E_used_Wh, 'g-',  'LineWidth', 2.2, 'DisplayName', 'Battery-Aware MPC');
    xlabel('Distance Traveled [m]'); ylabel('Cumulative Energy [Wh]');
    title('FIGURE 9: Cumulative Energy Consumption vs Distance Traveled');
    legend('Location', 'northwest'); grid on;
    saveas(fig9, fullfile(output_dir, 'fig9_cumulative_energy_vs_distance.png'));
    
    %% FIGURE 10: Control Actuation (Thrust & Yaw Moment)
    fig10 = figure('Name', 'Fig 10: Control Actuation', 'Position', [100 100 800 600], 'Color', 'w', 'Visible', 'off');
    subplot(2,1,1);
    plot(res3.t, res3.T, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Surge Thrust T [N]'); hold on;
    plot(res3.t, res3.T_L, 'g:', 'LineWidth', 1.2, 'DisplayName', 'Left Motor T_L');
    plot(res3.t, res3.T_R, 'm:', 'LineWidth', 1.2, 'DisplayName', 'Right Motor T_R');
    ylabel('Thrust [N]'); title('FIGURE 10: Propulsion Actuation & Slew Rate');
    legend('Location', 'best'); grid on;
    
    subplot(2,1,2);
    plot(res3.t, res3.N, 'r-', 'LineWidth', 1.8);
    xlabel('Time [s]'); ylabel('Yaw Moment N [N*m]'); grid on;
    saveas(fig10, fullfile(output_dir, 'fig10_control_actuation.png'));
    
    %% FIGURE 11: Route Comparison (A* Shortest vs Energy Route)
    fig11 = figure('Name', 'Fig 11: Route Comparison', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    subplot(1,2,1);
    plot_map_obstacles(res3.map_data); hold on;
    plot(res11.route.path(:,1), res11.route.path(:,2), 'r-o', 'LineWidth', 1.8, 'DisplayName', 'Shortest Route');
    plot(res3.route.path(:,1),  res3.route.path(:,2),  'g-s', 'LineWidth', 1.8, 'DisplayName', 'Energy Route');
    title('Candidate Route Trajectories'); axis equal; grid on; legend('Location', 'northwest');
    
    subplot(1,2,2);
    c_names = {'Distance [hm]', 'Energy [x10 Wh]', 'Time [x50 s]'};
    rA_vals = [res11.route.distance/100, res11.route.energy_Wh/10, res11.route.travel_time/50];
    rB_vals = [res3.route.distance/100,  res3.route.energy_Wh/10,  res3.route.travel_time/50];
    b_bar = bar([rA_vals; rB_vals]');
    b_bar(1).FaceColor = [0.8 0.2 0.2];
    b_bar(2).FaceColor = [0.2 0.7 0.2];
    set(gca, 'XTickLabel', c_names);
    legend('Shortest Path', 'Energy Path', 'Location', 'best');
    title('Multi-Objective Metrics'); grid on;
    saveas(fig11, fullfile(output_dir, 'fig11_route_comparison.png'));
    
    %% FIGURE 12: Mission State Machine Timeline
    fig12 = figure('Name', 'Fig 12: Mission State Timeline', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    stairs(res3.t, res3.mode, 'b-', 'LineWidth', 2.0);
    yticks([1 2 3 4 5]);
    yticklabels({'1: CRUISE', '2: ECONOMY', '3: DOCK', '4: RETURN-BASE', '5: EMERGENCY'});
    xlabel('Time [s]'); title('FIGURE 12: Supervisory Decision Mode Transitions');
    grid on; ylim([0.5 5.5]);
    saveas(fig12, fullfile(output_dir, 'fig12_mission_state_timeline.png'));
    
    %% FIGURE 13: Multi-SOC Degradation Comparison (80%, 60%, 40%, 20%)
    fig13 = figure('Name', 'Fig 13: Multi-SOC Comparison', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res3.t, res3.SOC, 'g-',  'LineWidth', 2.0, 'DisplayName', '80% Initial SOC'); hold on;
    plot(res4.t, res4.SOC, 'b-',  'LineWidth', 2.0, 'DisplayName', '60% Initial SOC');
    plot(res5.t, res5.SOC, 'm-',  'LineWidth', 2.0, 'DisplayName', '40% Initial SOC');
    plot(res6.t, res6.SOC, 'r--', 'LineWidth', 2.2, 'DisplayName', '20% Initial SOC (Preserved)');
    yline(20, 'k:', 'Reserve Buffer');
    xlabel('Time [s]'); ylabel('Battery SOC [%]');
    title('FIGURE 13: Battery-Aware MPC Adaptation Across Initial SOC Scenarios');
    legend('Location', 'best'); grid on;
    saveas(fig13, fullfile(output_dir, 'fig13_battery_aware_multisoc.png'));
    
    %% FIGURE 14: Environmental Disturbances Impact
    fig14 = figure('Name', 'Fig 14: Environmental Disturbances', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    plot(res3.t,  res3.P,  'k-',  'LineWidth', 1.6, 'DisplayName', 'Calm Water (Baseline)'); hold on;
    plot(res7.t,  res7.P,  'r--', 'LineWidth', 1.6, 'DisplayName', 'Opposing Current (0.45 m/s)');
    plot(res8.t,  res8.P,  'm-.', 'LineWidth', 1.6, 'DisplayName', 'Cross Current (0.40 m/s)');
    plot(res9.t,  res9.P,  'c:',  'LineWidth', 1.6, 'DisplayName', 'Wind (6.0 m/s)');
    plot(res10.t, res10.P, 'b-',  'LineWidth', 2.0, 'DisplayName', 'Combined Current + Wind + Waves');
    xlabel('Time [s]'); ylabel('Total Power [W]');
    title('FIGURE 14: Power Demand Under Marine Environmental Disturbances');
    legend('Location', 'best'); grid on;
    saveas(fig14, fullfile(output_dir, 'fig14_environmental_disturbances.png'));
    
    %% FIGURE 15: Controller Trade-Off Bar Summary
    fig15 = figure('Name', 'Fig 15: Controller Trade-Offs', 'Position', [100 100 800 500], 'Color', 'w', 'Visible', 'off');
    met_names = {'Energy [Wh]', 'Travel Time [s]', 'Speed RMSE [x100 m/s]', 'Cross-Track RMSE [x10 m]'};
    val_pid  = [res1.summary.energy_Wh, res1.summary.travel_time_s, res1.summary.rmse_u*100, res1.summary.rmse_cte*10];
    val_conv = [res2.summary.energy_Wh, res2.summary.travel_time_s, res2.summary.rmse_u*100, res2.summary.rmse_cte*10];
    val_ba   = [res3.summary.energy_Wh, res3.summary.travel_time_s, res3.summary.rmse_u*100, res3.summary.rmse_cte*10];
    
    b15 = bar([val_pid; val_conv; val_ba]');
    b15(1).FaceColor = [0.85 0.25 0.25];
    b15(2).FaceColor = [0.25 0.45 0.85];
    b15(3).FaceColor = [0.20 0.75 0.30];
    set(gca, 'XTickLabel', met_names);
    legend('PID Controller', 'Conventional MPC', 'Battery-Aware MPC', 'Location', 'best');
    title('FIGURE 15: Quantitative Trade-Off Comparison (PID vs Conv MPC vs Battery-Aware MPC)');
    grid on;
    saveas(fig15, fullfile(output_dir, 'fig15_controller_comparison.png'));
    
    fprintf('All 15 figures successfully generated and exported as PNGs!\n');
end

function plot_map_obstacles(map_data)
    for k = 1:length(map_data.obstacles)
        obs = map_data.obstacles(k);
        if strcmp(obs.type, 'circle')
            theta_c = linspace(0, 2*pi, 100);
            cx = obs.center(1) + obs.radius * cos(theta_c);
            cy = obs.center(2) + obs.radius * sin(theta_c);
            plot(cx, cy, '-', 'Color', [0.6 0.3 0.1], 'LineWidth', 1.5, 'HandleVisibility', 'off');
            fill(cx, cy, [0.8 0.6 0.4], 'FaceAlpha', 0.4, 'EdgeColor', [0.6 0.3 0.1], 'LineWidth', 1.5, 'HandleVisibility', 'off');
        elseif strcmp(obs.type, 'polygon')
            fill(obs.vertices(:,1), obs.vertices(:,2), [0.8 0.6 0.4], 'FaceAlpha', 0.5, 'EdgeColor', [0.5 0.3 0.1], 'LineWidth', 1.5);
        end
    end
    
    % Plot Key Nodes
    nodes = map_data.nodes;
    plot(nodes.base.x, nodes.base.y, 'gp', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Base (Start)');
    plot(nodes.dest.x, nodes.dest.y, 'rp', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'Destination');
    plot(nodes.dock.x, nodes.dock.y, 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm', 'DisplayName', 'Charging Dock');
    plot(nodes.harbor.x, nodes.harbor.y, 'co', 'MarkerSize', 9, 'MarkerFaceColor', 'c', 'DisplayName', 'Safe Harbor');
end

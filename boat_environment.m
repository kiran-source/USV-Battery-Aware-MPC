
clc;
clear;
close all;

%% Environment dimensions (meters)
mapLength = 1000;
mapWidth = 600;
resolution = 2;

x = 0:resolution:mapLength;
y = 0:resolution:mapWidth;
[X, Y] = meshgrid(x, y);

%% Create a winding waterway
% The centerline bends along the map.
centerY = 300 ...
    + 65*sin(2*pi*X/mapLength) ...
    + 25*sin(4*pi*X/mapLength);

% Waterway width varies along its length.
channelWidth = 190 + 20*sin(2*pi*X/mapLength);

% Free water = 0; land/restricted area = 1
occupancyMap = abs(Y - centerY) > channelWidth/2;

%% Add an island in the waterway
island = ((X - 510)/55).^2 + ((Y - 330)/38).^2 <= 1;
occupancyMap(island) = 1;

%% Add restricted areas / floating obstacles
obstacles = [
    250 260 15
    370 365 18
    680 250 20
    800 350 16
];

for k = 1:size(obstacles, 1)
    ox = obstacles(k, 1);
    oy = obstacles(k, 2);
    r  = obstacles(k, 3);

    obstacleMask = (X - ox).^2 + (Y - oy).^2 <= r^2;
    occupancyMap(obstacleMask) = 1;
end

%% Define start and destination
startPoint = [40, 300];
goalPoint  = [950, 300];

%% Display the environment
figure('Color', 'w');
imagesc(x, y, occupancyMap);
set(gca, 'YDir', 'normal');
axis equal tight;
colormap([0.90 0.95 1.00; 0.45 0.38 0.30]);
hold on;
grid on;
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);

plot(startPoint(1), startPoint(2), 'go', ...
    'MarkerFaceColor', [0.1 0.7 0.2], 'MarkerSize', 10);

plot(goalPoint(1), goalPoint(2), 'r*', ...
    'MarkerSize', 14, 'LineWidth', 2);

title('Simulated Tourist Boat Water Environment', 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
xlabel('X Position (m)', 'Color', [0.15 0.15 0.15]);
ylabel('Y Position (m)', 'Color', [0.15 0.15 0.15]);
legend('Start', 'Destination', 'Location', 'best', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);

%% Save environment data for the path planner
save('boat_environment_data.mat', ...
    'occupancyMap', 'x', 'y', 'resolution', ...
    'startPoint', 'goalPoint');

disp('Environment created and saved successfully.');

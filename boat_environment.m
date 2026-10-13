
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
colormap([0.2 0.65 0.85; 0.35 0.35 0.35]);

hold on;
plot(startPoint(1), startPoint(2), 'go', ...
    'MarkerFaceColor', 'g', 'MarkerSize', 10);

plot(goalPoint(1), goalPoint(2), 'r*', ...
    'MarkerSize', 12, 'LineWidth', 2);

title('Simulated Tourist Boat Water Environment');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Start', 'Destination', 'Location', 'best');
grid on;

%% Save environment data for the path planner
save('boat_environment_data.mat', ...
    'occupancyMap', 'x', 'y', 'resolution', ...
    'startPoint', 'goalPoint');

disp('Environment created and saved successfully.');

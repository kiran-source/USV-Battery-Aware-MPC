
clc;
clear;
close all;

% Create a 100-by-100 map
grid = zeros(100,100);

% Add rectangular obstacles
grid(25:65, 35:42) = 1;
grid(45:85, 65:72) = 1;
grid(10:20, 75:90) = 1;

% Start and goal positions [row, column]
start = [5 5];
goal = [90 90];

% Run Theta*
path = theta_star(grid, start, goal);

% Display map and planned route
figure('Color', 'w');
imagesc(grid);
colormap([1 1 1; 0.30 0.35 0.42]);
axis equal tight;
axis ij;
hold on;
grid on;
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);

if ~isempty(path)
    plot(path(:,2), path(:,1), 'b-', 'LineWidth', 2);
    plot(start(2), start(1), 'go', ...
        'MarkerFaceColor', 'g', 'MarkerSize', 9);
    plot(goal(2), goal(1), 'ro', ...
        'MarkerFaceColor', 'r', 'MarkerSize', 9);
    title('Theta* Planned Path');
    legend('Planned path','Start','Goal');
else
    title('No path found');
end

xlabel('Column');
ylabel('Row');

% Display path length
if ~isempty(path)
    segmentLengths = sqrt(sum(diff(path).^2,2));
    fprintf('Number of path points: %d\n', size(path,1));
    fprintf('Total path length: %.2f cells\n', ...
        sum(segmentLengths));
end

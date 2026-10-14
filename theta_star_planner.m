
clc;
close all;

%% Load environment
load('boat_environment_data.mat');

% Grid dimensions
[nRows, nCols] = size(occupancyMap);

% Convert world coordinates to grid indices
worldToGrid = @(p) [ ...
    round(p(2)/resolution) + 1, ...
    round(p(1)/resolution) + 1];

start = worldToGrid(startPoint);
goal  = worldToGrid(goalPoint);

%% Theta* setup
gScore = inf(nRows, nCols);
parentRow = zeros(nRows, nCols);
parentCol = zeros(nRows, nCols);
closed = false(nRows, nCols);

gScore(start(1), start(2)) = 0;

openList = start;
found = false;

% 8-connected movement
moves = [
    -1  0
     1  0
     0 -1
     0  1
    -1 -1
    -1  1
     1 -1
     1  1
];

%% Theta* search
while ~isempty(openList)

    % Select node with lowest estimated total cost
    costs = zeros(size(openList,1),1);

    for k = 1:size(openList,1)
        r = openList(k,1);
        c = openList(k,2);

        h = hypot(r-goal(1), c-goal(2));
        costs(k) = gScore(r,c) + h;
    end

    [~, idx] = min(costs);
    current = openList(idx,:);
    openList(idx,:) = [];

    r = current(1);
    c = current(2);

    if closed(r,c)
        continue;
    end

    closed(r,c) = true;

    if isequal(current,goal)
        found = true;
        break;
    end

    for k = 1:size(moves,1)
        nr = r + moves(k,1);
        nc = c + moves(k,2);

        if nr < 1 || nr > nRows || nc < 1 || nc > nCols
            continue;
        end

        if occupancyMap(nr,nc) || closed(nr,nc)
            continue;
        end

        % Try connecting the neighbor directly to current's parent
        pr = parentRow(r,c);
        pc = parentCol(r,c);

        if pr > 0 && lineOfSight( ...
                occupancyMap, pr, pc, nr, nc)

            newCost = gScore(pr,pc) + ...
                hypot(nr-pr, nc-pc)*resolution;

            if newCost < gScore(nr,nc)
                gScore(nr,nc) = newCost;
                parentRow(nr,nc) = pr;
                parentCol(nr,nc) = pc;
            end

        else
            % Standard neighbor connection
            if ~lineOfSight(occupancyMap, r, c, nr, nc)
                continue;
            end

            newCost = gScore(r,c) + ...
                hypot(nr-r, nc-c)*resolution;

            if newCost < gScore(nr,nc)
                gScore(nr,nc) = newCost;
                parentRow(nr,nc) = r;
                parentCol(nr,nc) = c;
            end
        end

        if ~any(openList(:,1)==nr & openList(:,2)==nc)
            openList(end+1,:) = [nr nc];
        end
    end
end

%% Reconstruct path
if ~found
    error('No collision-free path found.');
end

pathGrid = goal;

r = goal(1);
c = goal(2);

while ~(r == start(1) && c == start(2))
    pr = parentRow(r,c);
    pc = parentCol(r,c);

    if pr == 0 || pc == 0
        error('Path reconstruction failed.');
    end

    pathGrid(end+1,:) = [pr pc];
    r = pr;
    c = pc;
end

pathGrid = flipud(pathGrid);

% Convert grid indices to world coordinates [x,y]
pathXY = [(pathGrid(:,2)-1)*resolution, ...
          (pathGrid(:,1)-1)*resolution];

%% Plot path
figure('Color','w');
imagesc(x,y,occupancyMap);
set(gca,'YDir','normal');
axis equal tight;
colormap([0.90 0.95 1.00; 0.45 0.38 0.30]); % light water, slate obstacles
hold on;
grid on;
set(gca, 'Color', 'w', 'XColor', [0.15 0.15 0.15], 'YColor', [0.15 0.15 0.15], 'GridColor', [0.85 0.85 0.85], 'GridAlpha', 0.5);

plot(startPoint(1),startPoint(2),'go', ...
    'MarkerFaceColor',[0.1 0.7 0.2],'MarkerSize',10);

plot(goalPoint(1),goalPoint(2),'r*', ...
    'MarkerSize',14,'LineWidth',2);

plot(pathXY(:,1),pathXY(:,2),'-', ...
    'Color', [0.85 0.20 0.10], 'LineWidth', 2.5);

title('Theta* Automatically Generated Path', 'FontWeight', 'bold', 'Color', [0.1 0.1 0.1]);
xlabel('X Position (m)', 'Color', [0.15 0.15 0.15]);
ylabel('Y Position (m)', 'Color', [0.15 0.15 0.15]);
legend('Start','Destination','Theta* Path', 'Location', 'best', 'Color', 'w', 'EdgeColor', [0.8 0.8 0.8]);

% Save generated route
save('theta_star_path.mat','pathXY','startPoint','goalPoint');

disp('Theta* path generated successfully.');
fprintf('Number of path points: %d\n',size(pathXY,1));
fprintf('Path length: %.2f m\n', ...
    sum(vecnorm(diff(pathXY),2,2)));

%% Line-of-sight helper
function visible = lineOfSight(map,r1,c1,r2,c2)
    n = max(abs(r2-r1),abs(c2-c1))*2 + 1;
    rows = round(linspace(r1,r2,n));
    cols = round(linspace(c1,c2,n));

    indices = sub2ind(size(map),rows,cols);
    visible = ~any(map(indices));
end

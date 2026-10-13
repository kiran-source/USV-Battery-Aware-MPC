
function path = theta_star(grid, start, goal)
% THETA* path planning on a 2D occupancy grid
% grid: 0 = free, 1 = obstacle
% start, goal: [row, column]
% path: N-by-2 array of [row, column]

[nRows, nCols] = size(grid);

% Validate inputs
if numel(start) ~= 2 || numel(goal) ~= 2 || ...
   any(start ~= round(start)) || any(goal ~= round(goal)) || ...
   any(start < 1) || start(1) > nRows || start(2) > nCols || ...
   any(goal < 1) || goal(1) > nRows || goal(2) > nCols
    error('Start and goal must be valid grid coordinates.');
end

if grid(start(1),start(2)) ~= 0 || ...
   grid(goal(1),goal(2)) ~= 0
    error('Start and goal must be in free cells.');
end

g = inf(nRows,nCols);
parent = zeros(nRows,nCols,2);
closed = false(nRows,nCols);

g(start(1),start(2)) = 0;
parent(start(1),start(2),:) = reshape(start,1,1,2);

open = start;

% 8-connected movement
moves = [-1 0; 1 0; 0 -1; 0 1; ...
         -1 -1; -1 1; 1 -1; 1 1];

while ~isempty(open)

    % Select node with lowest estimated total cost
    f = zeros(size(open,1),1);

    for k = 1:size(open,1)
        node = open(k,:);
        f(k) = g(node(1),node(2)) + ...
            hypot(goal(1)-node(1),goal(2)-node(2));
    end

    [~,idx] = min(f);
    current = open(idx,:);
    open(idx,:) = [];

    r = current(1);
    c = current(2);

    if closed(r,c)
        continue;
    end

    closed(r,c) = true;

    if isequal(current,goal)
        break;
    end

    for k = 1:size(moves,1)

        neighbor = current + moves(k,:);
        nr = neighbor(1);
        nc = neighbor(2);

        % Check grid boundaries
        if nr < 1 || nr > nRows || nc < 1 || nc > nCols
            continue;
        end

        % Skip obstacles and visited nodes
        if grid(nr,nc) ~= 0 || closed(nr,nc)
            continue;
        end

        % Prevent diagonal corner cutting
        dr = nr-r;
        dc = nc-c;

        if abs(dr) == 1 && abs(dc) == 1
            if grid(r+dr,c) ~= 0 || grid(r,c+dc) ~= 0
                continue;
            end
        end

        pr = parent(r,c,1);
        pc = parent(r,c,2);

        % Theta*: try connecting to the current node's parent
        if hasLOS(grid,[pr pc],neighbor)
            tentative = g(pr,pc) + hypot(nr-pr,nc-pc);
            newParent = [pr pc];
        else
            tentative = g(r,c) + hypot(nr-r,nc-c);
            newParent = current;
        end

        if tentative < g(nr,nc)
            g(nr,nc) = tentative;
            parent(nr,nc,:) = reshape(newParent,1,1,2);
            open = [open; neighbor];
        end
    end
end

% Check whether goal was reached
if ~closed(goal(1),goal(2))
    path = [];
    warning('No path found.');
    return;
end

% Reconstruct path
path = goal;
current = goal;

while ~isequal(current,start)

    p = reshape(parent(current(1),current(2),:),1,2);

    if all(p == 0) || isequal(p,current)
        path = [];
        warning('Path reconstruction failed.');
        return;
    end

    current = p;
    path = [current; path];
end

% Final path safety validation
for k = 1:size(path,1)-1
    if ~hasLOS(grid,path(k,:),path(k+1,:))
        path = [];
        warning('Path validation failed: obstacle intersection.');
        return;
    end
end

end


function visible = hasLOS(grid,a,b)
% Conservative grid line-of-sight check.
% Checks all grid cells touched by the line segment.

dr = b(1)-a(1);
dc = b(2)-a(2);

steps = max(abs(dr),abs(dc))*10;

if steps == 0
    visible = grid(a(1),a(2)) == 0;
    return;
end

visible = true;
previous = [-1 -1];

for i = 0:steps

    t = i/steps;
    r = round(a(1)+t*dr);
    c = round(a(2)+t*dc);

    if r < 1 || r > size(grid,1) || ...
       c < 1 || c > size(grid,2) || grid(r,c) ~= 0
        visible = false;
        return;
    end

    % Check diagonal transitions for corner cutting
    if all(previous > 0)
        rr = r-previous(1);
        cc = c-previous(2);

        if abs(rr) == 1 && abs(cc) == 1
            if grid(previous(1)+rr,previous(2)) ~= 0 || ...
               grid(previous(1),previous(2)+cc) ~= 0
                visible = false;
                return;
            end
        end
    end

    previous = [r c];
end

end

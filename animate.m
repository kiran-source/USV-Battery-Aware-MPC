clc;
close all;

%% =========================================================
% LOAD ENVIRONMENT AND THETA* PATH
% =========================================================

load('boat_environment_data.mat');
load('theta_star_path.mat');

%% =========================================================
% MISSION
% =========================================================

startPos    = [0 0];
destination = [980 100];

boatSpeed = 2.0;          % m/s
arrivalTolerance = 5.0;   % m
maxSimTime = 1100;        % s
dt = 0.5;                 % s

%% =========================================================
% MAKE SURE PATH IS IN THE SAME COORDINATE SYSTEM
% =========================================================

% Use the saved Theta* path.
pathXY = double(pathXY);

% Force exact project start and destination.
pathXY(1,:)   = startPos;
pathXY(end,:) = destination;

%% =========================================================
% OCCUPANCY MAP
% =========================================================

[mapRows,mapCols] = size(occupancyMap);

% Mission coordinate system
missionXMin = 0;
missionXMax = 1000;

% The Theta* mission is close to y = 0...100.
% Place the environment around this mission corridor.
missionYMin = -100;
missionYMax = 900;

mapX = linspace( ...
    missionXMin, ...
    missionXMax, ...
    mapCols);

mapY = linspace( ...
    missionYMin, ...
    missionYMax, ...
    mapRows);

%% =========================================================
% INITIAL STATE
% =========================================================

position = startPos;

simTime = 0;

arrived = false;

%% =========================================================
% FIGURE
% =========================================================

figure( ...
    'Color','w', ...
    'Position',[100 100 1150 700]);

imagesc( ...
    mapX, ...
    mapY, ...
    occupancyMap);

set(gca,'YDir','normal');

hold on;

axis equal;

xlim([-50 1050]);
ylim([-100 900]);

%% Map colors

colormap([
    0.20 0.65 0.85
    0.35 0.35 0.35
]);

%% =========================================================
% THETA* PLANNED PATH
% =========================================================

plot( ...
    pathXY(:,1), ...
    pathXY(:,2), ...
    'y--', ...
    'LineWidth',2.5, ...
    'DisplayName','Theta* Planned Path');

%% =========================================================
% START
% =========================================================

plot( ...
    startPos(1), ...
    startPos(2), ...
    'go', ...
    'MarkerFaceColor','g', ...
    'MarkerSize',10, ...
    'DisplayName','Start');

%% =========================================================
% DESTINATION
% =========================================================

plot( ...
    destination(1), ...
    destination(2), ...
    'r*', ...
    'MarkerSize',14, ...
    'LineWidth',2, ...
    'DisplayName','Destination');

%% =========================================================
% BOAT
% =========================================================

boatShape = [
     9   0
    -9  -4
    -5   0
    -9   4
];

boat = patch( ...
    boatShape(:,1) + position(1), ...
    boatShape(:,2) + position(2), ...
    [1 0.25 0.05], ...
    'EdgeColor','k', ...
    'LineWidth',1.5, ...
    'DisplayName','USV');

%% =========================================================
% ACTUAL TRAJECTORY
% =========================================================

trail = animatedline( ...
    'Color','w', ...
    'LineWidth',2, ...
    'DisplayName','Actual Trajectory');

%% =========================================================
% AXIS
% =========================================================

xlabel('X Position (m)');
ylabel('Y Position (m)');

title('Theta* USV Navigation and Obstacle Avoidance');

grid on;

legend('Location','northwest');

%% =========================================================
% PATH FOLLOWING
% =========================================================

segment = 1;

pathCount = size(pathXY,1);

while simTime < maxSimTime

    %% -----------------------------------------------------
    % DISTANCE TO DESTINATION
    % ------------------------------------------------------

    distanceToGoal = ...
        norm(destination - position);

    if distanceToGoal <= arrivalTolerance

        arrived = true;

        break;

    end

    %% -----------------------------------------------------
    % FIND NEXT WAYPOINT
    % ------------------------------------------------------

    while segment < pathCount && ...
            norm(pathXY(segment,:) - position) < 2

        segment = segment + 1;

    end

    %% -----------------------------------------------------
    % CURRENT TARGET
    % ------------------------------------------------------

    target = pathXY(segment,:);

    direction = target - position;

    distanceToTarget = norm(direction);

    if distanceToTarget < 1e-9

        segment = min( ...
            segment + 1, ...
            pathCount);

        continue;

    end

    %% Normalize direction

    direction = ...
        direction / distanceToTarget;

    %% -----------------------------------------------------
    % MOVE BOAT
    % ------------------------------------------------------

    stepDistance = min( ...
        boatSpeed * dt, ...
        distanceToTarget);

    position = ...
        position + ...
        direction * stepDistance;

    simTime = simTime + dt;

    %% -----------------------------------------------------
    % BOAT HEADING
    % ------------------------------------------------------

    heading = atan2( ...
        direction(2), ...
        direction(1));

    R = [
        cos(heading) -sin(heading)
        sin(heading)  cos(heading)
    ];

    rotatedBoat = ...
        (R * boatShape')';

    %% -----------------------------------------------------
    % UPDATE BOAT GRAPHIC
    % ------------------------------------------------------

    set( ...
        boat, ...
        'XData', ...
        rotatedBoat(:,1) + position(1), ...
        'YData', ...
        rotatedBoat(:,2) + position(2));

    %% -----------------------------------------------------
    % UPDATE ACTUAL TRAIL
    % ------------------------------------------------------

    addpoints( ...
        trail, ...
        position(1), ...
        position(2));

    %% -----------------------------------------------------
    % UPDATE TITLE
    % ------------------------------------------------------

    distanceToGoal = ...
        norm(destination-position);

    title(sprintf( ...
        'Theta* USV Navigation | Time: %.1f s | Distance Remaining: %.1f m', ...
        simTime, ...
        distanceToGoal));

    drawnow limitrate;

    pause(0.01);

end

%% =========================================================
% FINAL RESULT
% =========================================================

finalDistance = ...
    norm(destination-position);

fprintf('\n');
fprintf('============================================\n');
fprintf('        THETA* USV NAVIGATION RESULT\n');
fprintf('============================================\n');

if arrived

    fprintf('MISSION COMPLETED\n');

else

    fprintf('MISSION NOT COMPLETED\n');

end

fprintf('Simulation time    : %.2f s\n',simTime);

fprintf( ...
    'Final position     : (%.2f, %.2f) m\n', ...
    position(1), ...
    position(2));

fprintf( ...
    'Distance remaining : %.2f m\n', ...
    finalDistance);

fprintf( ...
    'Arrival status     : %d\n', ...
    arrived);

fprintf('============================================\n');
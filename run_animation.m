%% =========================================================================
%  LAUNCHER: 2D ANIMATED USV HYDRODYNAMICS SIMULATOR
%  Runs visual animation of the boat navigating through waypoints, obstacles,
%  ocean waves, and current, showing real-time Battery-Aware MPC telemetry.
% =========================================================================

clear; clc;

addpath(genpath(pwd));

fprintf('Loading Battery-Aware MPC simulation dataset...\n');
if exist('simulation_results.mat', 'file')
    data = load('simulation_results.mat');
    res = data.all_results{3}; % Exp 3: Battery-Aware MPC (80% Start)
else
    fprintf('Running simulation...\n');
    p = load_all_parameters();
    res = run_simulation(p);
end

fprintf('Starting 2D Visual Marine Animation...\n');
% 8x playback speed for smooth and fast viewing
animate_USV_mission(res, 8.0);

%% =========================================================================
%  BATTERY-AWARE MODEL PREDICTIVE SPEED AND PATH CONTROL FOR AN AUTONOMOUS
%  SURFACE VEHICLE (USV) WITH ENERGY-AWARE ROUTE PLANNING & 3-DOF HYDRODYNAMICS
%
%  Master Execution & Evaluation Suite (15 Benchmark Experiments)
% =========================================================================

clear; clc; close all;

fprintf('========================================================================\n');
fprintf('  STARTING BATTERY-AWARE 3-DOF USV MPC & ROUTE PLANNING SUITE           \n');
fprintf('========================================================================\n\n');

% Ensure all subdirectories are on the MATLAB path
root_dir = fileparts(mfilename('fullpath'));
addpath(genpath(root_dir));

%% 1. LOAD SYSTEM PARAMETERS
fprintf('[Step 1/5] Loading centralized parameters...\n');
p = load_all_parameters();

%% 2. RUN ALL 15 BENCHMARK EXPERIMENTS
fprintf('[Step 2/5] Executing 15 benchmark experiments...\n');
t_start = tic;
all_results = run_all_experiments();
t_elapsed = toc(t_start);
fprintf('All 15 experiments executed in %.2f seconds.\n\n', t_elapsed);

%% 3. GENERATE ALL 15 PUBLICATION FIGURES
fprintf('[Step 3/5] Generating all 15 figures...\n');
generate_all_figures(all_results, root_dir);

%% 4. GENERATE TELEMETRY DASHBOARD
fprintf('[Step 4/5] Generating mission dashboard for Battery-Aware MPC...\n');
create_dashboard(all_results{3}, root_dir);

%% 5. GENERATE & EXPORT COMPARISON METRICS TABLE
fprintf('[Step 5/5] Exporting summary metrics table...\n');
summary_tbl = generate_summary_table(all_results, fullfile(root_dir, 'final_results_summary.csv'));

% Save workspace results
if ~exist(fullfile(root_dir, 'results'), 'dir')
    mkdir(fullfile(root_dir, 'results'));
end
save(fullfile(root_dir, 'results', 'all_simulation_results.mat'), 'all_results', 'summary_tbl', 'p');
save(fullfile(root_dir, 'simulation_results.mat'), 'all_results', 'summary_tbl', 'p');

fprintf('\n========================================================================\n');
fprintf('  FINAL EVALUATION SUITE COMPLETED SUCCESSFULLY!                         \n');
fprintf('  Results saved to: final_results_summary.csv & results/                 \n');
fprintf('  Figures saved to: fig1_2D_trajectory.png to fig15_... & dashboard.png   \n');
fprintf('========================================================================\n');

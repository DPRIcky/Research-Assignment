%% AUTONOMOUS VEHICLE NAVIGATION - Interactive Mode
% This script prompts the user to select configuration options

clear all; close all; clc;

% Add paths
addpath('planners');
addpath('controllers');
addpath('safety');
addpath('utils');
addpath('models');

fprintf('=========================================================\n');
fprintf('  AUTONOMOUS VEHICLE NAVIGATION SIMULATION\n');
fprintf('  Interactive Configuration Mode\n');
fprintf('=========================================================\n\n');

%% User Prompts for Configuration

% Planner selection
fprintf('Available Path Planners:\n');
fprintf('  1. A* (Grid-based search)\n');
fprintf('  2. RRT* (Sampling-based with rewiring)\n');
fprintf('  3. Hybrid A* (Kinematic planning)\n');
planner = input('Select planner (1-3): ');

while ~ismember(planner, [1, 2, 3])
    fprintf('Invalid selection. Please choose 1, 2, or 3.\n');
    planner = input('Select planner (1-3): ');
end

% Controller selection
fprintf('\nAvailable Controllers:\n');
fprintf('  1. PID (Proportional-Integral-Derivative)\n');
fprintf('  2. LQR (Linear Quadratic Regulator)\n');
fprintf('  3. MPC (Model Predictive Control)\n');
controller = input('Select controller (1-3): ');

while ~ismember(controller, [1, 2, 3])
    fprintf('Invalid selection. Please choose 1, 2, or 3.\n');
    controller = input('Select controller (1-3): ');
end

% CBF safety layer
fprintf('\nControl Barrier Function (CBF) Safety Layer:\n');
cbf_choice = input('Enable CBF safety? (y/n): ', 's');
cbf_enabled = strcmpi(cbf_choice, 'y') || strcmpi(cbf_choice, 'yes');

% State estimator selection
fprintf('\nAvailable State Estimators:\n');
fprintf('  1. Perfect (Ground truth)\n');
fprintf('  2. Noisy (Gaussian noise)\n');
fprintf('  3. EKF (Extended Kalman Filter)\n');
fprintf('  4. External (Use provided data)\n');
estimator = input('Select state estimator (1-4): ');

while ~ismember(estimator, [1, 2, 3, 4])
    fprintf('Invalid selection. Please choose 1, 2, 3, or 4.\n');
    estimator = input('Select state estimator (1-4): ');
end

fprintf('\n');

%% Run the main simulation with selected configuration
% Set configuration variables and run main script
run('run_simulation.m');

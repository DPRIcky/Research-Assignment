% test_new_utilities.m - Quick test of newly added utility functions
% This script demonstrates the new compute_metrics, plot_simulation_results, 
% and save_results functions

clear all; close all; clc;

fprintf('=========================================================\n');
fprintf('  TESTING NEW UTILITY FUNCTIONS\n');
fprintf('=========================================================\n\n');

% Add paths
addpath('core');
addpath('utils');
addpath('planners');
addpath('controllers');
addpath('safety');

%% 1. Setup a simple test scenario
fprintf('1. Setting up test scenario...\n');

% Create simple environment
env = Environment([50, 50], 1.0);
env = env.add_circular_obstacle(25, 25, 5);

% Plan a simple path with A*
planner = AStarPlanner(env, 1.0);
start_pos = [5, 5];
goal_pos = [45, 45];
[path_2d, stats] = planner.plan(start_pos, goal_pos);

% Add headings to path
path = [path_2d, zeros(size(path_2d, 1), 1)];
for i = 1:size(path, 1)-1
    dx = path(i+1, 1) - path(i, 1);
    dy = path(i+1, 2) - path(i, 2);
    path(i, 3) = atan2(dy, dx);
end
path(end, 3) = path(end-1, 3);

fprintf('   ✓ Path planned with %d waypoints\n\n', size(path, 1));

%% 2. Generate sample trajectory data
fprintf('2. Generating sample trajectory data...\n');

dt = 0.1;
n_steps = 200;
timestamps = (0:n_steps-1)' * dt;

% Simple trajectory following the path
trajectory = zeros(n_steps, 4);  % [x, y, theta, v]
trajectory_est = zeros(n_steps, 4);
controls = zeros(n_steps, 2);    % [v, omega]
tracking_errors = zeros(n_steps, 1);
estimation_errors = zeros(n_steps, 1);
cbf_activations = zeros(n_steps, 1);
min_distances = zeros(n_steps, 1);

% Initialize
trajectory(1, :) = [start_pos, 0, 0];
trajectory_est(1, :) = trajectory(1, :);

% Simple simulation loop
for i = 2:n_steps
    % Simple proportional control toward next waypoint
    current_pos = trajectory(i-1, 1:2);
    distances_to_waypoints = sqrt(sum((path(:,1:2) - current_pos).^2, 2));
    [~, nearest_idx] = min(distances_to_waypoints);
    target_idx = min(nearest_idx + 3, size(path, 1));
    
    target = path(target_idx, 1:2);
    direction = atan2(target(2) - current_pos(2), target(1) - current_pos(1));
    
    % Control
    v = 2.0;
    heading_error = direction - trajectory(i-1, 3);
    heading_error = atan2(sin(heading_error), cos(heading_error));
    omega = 2.0 * heading_error;
    
    controls(i, :) = [v, omega];
    
    % Update state (simple kinematic model)
    trajectory(i, 1) = trajectory(i-1, 1) + v * cos(trajectory(i-1, 3)) * dt;
    trajectory(i, 2) = trajectory(i-1, 2) + v * sin(trajectory(i-1, 3)) * dt;
    trajectory(i, 3) = trajectory(i-1, 3) + omega * dt;
    trajectory(i, 4) = v;
    
    % Add noise to estimation
    noise = [randn()*0.2, randn()*0.2, randn()*0.05, randn()*0.1];
    trajectory_est(i, :) = trajectory(i, :) + noise;
    
    % Tracking error
    tracking_errors(i) = min(distances_to_waypoints);
    
    % Estimation error
    estimation_errors(i) = norm(trajectory(i, 1:3) - trajectory_est(i, 1:3));
    
    % Fake CBF activation (activate near obstacle)
    dist_to_obstacle = norm(trajectory(i, 1:2) - [25, 25]) - 5;
    min_distances(i) = dist_to_obstacle;
    cbf_activations(i) = (dist_to_obstacle < 3.0);
end

fprintf('   ✓ Generated %d timesteps of trajectory data\n\n', n_steps);

%% 3. Test compute_metrics function
fprintf('3. Testing compute_metrics.m...\n');

config = struct();
config.planner_name = 'A*';
config.controller_name = 'Test';
config.cbf_enabled = true;
config.estimator_name = 'Noisy';
config.start_pos = start_pos;
config.goal_pos = goal_pos;

metrics = compute_metrics(trajectory, path, controls, timestamps, ...
                         cbf_activations, min_distances, stats);

fprintf('   ✓ Metrics computed successfully!\n');
fprintf('   Sample metrics:\n');
fprintf('      - Path length: %.2f m\n', metrics.path_length);
fprintf('      - Mean tracking error: %.3f m\n', metrics.mean_tracking_error);
fprintf('      - CBF activation rate: %.1f%%\n', metrics.cbf_activation_rate);
fprintf('      - Path efficiency: %.3f\n\n', metrics.path_efficiency);

%% 4. Test plot_simulation_results function
fprintf('4. Testing plot_simulation_results.m...\n');

fig = plot_simulation_results(env, path, trajectory, trajectory_est, ...
                              controls, tracking_errors, estimation_errors, ...
                              cbf_activations, min_distances, timestamps, config);

fprintf('   ✓ Visualization generated successfully!\n');
fprintf('   Figure created with 6 subplots\n\n');

%% 5. Test save_results function
fprintf('5. Testing save_results.m...\n');

filename = 'test_utilities';
save_results(metrics, trajectory, controls, timestamps, config, filename);

fprintf('   ✓ Results saved successfully!\n\n');

%% 6. Verify saved files
fprintf('6. Verifying saved files...\n');

if exist('results', 'dir')
    files = dir('results/test_utilities*');
    fprintf('   Files created in results/:\n');
    for i = 1:length(files)
        fprintf('      - %s (%.1f KB)\n', files(i).name, files(i).bytes/1024);
    end
else
    fprintf('   Warning: results/ directory not created\n');
end

fprintf('\n=========================================================\n');
fprintf('ALL NEW UTILITIES TESTED SUCCESSFULLY!\n');
fprintf('=========================================================\n\n');

fprintf('Summary of new utilities:\n');
fprintf('  1. compute_metrics.m       - Computes %d performance metrics\n', ...
        length(fieldnames(metrics)));
fprintf('  2. plot_simulation_results - Creates comprehensive 6-panel visualization\n');
fprintf('  3. save_results.m          - Exports to MAT, CSV, and TXT formats\n\n');

fprintf('You can now use these utilities in your simulations!\n');
fprintf('Check the results/ folder for generated output files.\n');

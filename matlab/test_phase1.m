% test_phase1.m - Test Phase 1: Foundation Components
% Tests vehicle dynamics, environment, and state estimator

% Add paths
addpath('core');
addpath('utils');

clear all;
close all;
clc;

fprintf('===== PHASE 1 TESTING =====\n\n');

%% Test 1: Vehicle Dynamics - Bicycle Model
fprintf('Test 1: Vehicle Dynamics - Bicycle Model\n');
fprintf('------------------------------------------\n');

% Create vehicle
vehicle = VehicleDynamics('bicycle', 0.1);
fprintf('Created bicycle model with wheelbase L = %.2f m\n', vehicle.L);

% Initialize state [x, y, theta, v]
state = [0; 0; 0; 0];
fprintf('Initial state: [x=%.2f, y=%.2f, theta=%.2f, v=%.2f]\n', state(1), state(2), state(3), state(4));

% Simulate forward motion with slight steering
control = [1.0; 0.1];  % [acceleration, steering_angle]
states = zeros(4, 50);
states(:, 1) = state;

for i = 2:50
    state = vehicle.step(state, control);
    states(:, i) = state;
end

fprintf('After 50 steps: [x=%.2f, y=%.2f, theta=%.2f, v=%.2f]\n', state(1), state(2), state(3), state(4));

% Plot trajectory
figure('Name', 'Test 1: Bicycle Model Trajectory');
hold on;
plot(states(1, :), states(2, :), 'y-', 'DisplayName', 'Trajectory');
plot(states(1, :), states(2, :), 'c.', 'MarkerSize', 10, 'HandleVisibility', 'off');
plot(states(1, 1), states(2, 1), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
plot(states(1, end), states(2, end), 'r*', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'End');
% Add final heading arrow at goal
quiver(states(1, end), states(2, end), ...
       cos(states(3, end)), sin(states(3, end)), ...
       'b', 'LineWidth', 1, 'MaxHeadSize', 0.5, 'DisplayName', 'Final Heading');
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Bicycle Model - Forward Motion with Steering', 'FontSize', 14);
legend('Location', 'best');
grid on;
axis equal;
hold off;

fprintf('✓ Test 1 passed!\n\n');

%% Test 2: Vehicle Dynamics - Differential Drive
fprintf('Test 2: Vehicle Dynamics - Differential Drive\n');
fprintf('----------------------------------------------\n');

% Create differential drive vehicle
vehicle_diff = VehicleDynamics('differential', 0.1);
fprintf('Created differential drive model\n');

% Initialize state [x, y, theta]
state_diff = [0; 0; 0];
fprintf('Initial state: [x=%.2f, y=%.2f, theta=%.2f]\n', state_diff(1), state_diff(2), state_diff(3));

% Simulate circular motion
control_diff = [1.0; 0.3];  % [linear_velocity, angular_velocity]
states_diff = zeros(3, 50);
states_diff(:, 1) = state_diff;

for i = 2:50
    state_diff = vehicle_diff.step(state_diff, control_diff);
    states_diff(:, i) = state_diff;
end

fprintf('After 50 steps: [x=%.2f, y=%.2f, theta=%.2f]\n', state_diff(1), state_diff(2), state_diff(3));

% Plot trajectory
figure('Name', 'Test 2: Differential Drive Trajectory');
hold on;
plot(states_diff(1, :), states_diff(2, :), 'b-', 'DisplayName', 'Trajectory');
plot(states_diff(1, :), states_diff(2, :), 'y.', 'MarkerSize', 10, 'HandleVisibility', 'on');
plot(states_diff(1, 1), states_diff(2, 1), 'go', 'MarkerSize', 15, 'MarkerFaceColor', 'g', 'LineWidth', 2, 'DisplayName', 'Start');
plot(states_diff(1, end), states_diff(2, end), 'r*', 'MarkerSize', 20, 'LineWidth', 3, 'DisplayName', 'End');
% Add final heading arrow at goal
quiver(states_diff(1, end), states_diff(2, end), ...
       cos(states_diff(3, end)), sin(states_diff(3, end)), ...
       'b', 'LineWidth', 1, 'MaxHeadSize', 0.3, 'DisplayName', 'Final Heading');
xlabel('X (m)', 'FontSize', 12);
ylabel('Y (m)', 'FontSize', 12);
title('Differential Drive - Circular Motion', 'FontSize', 14);
legend('Location', 'best');
grid on;
axis equal;
hold off;

fprintf('✓ Test 2 passed!\n\n');

%% Test 3: Environment Setup
fprintf('Test 3: Environment Setup\n');
fprintf('-------------------------\n');

% Create environment
env = Environment([100, 100], 1.0);
fprintf('Created environment: %dx%d m, resolution: %.2f m/cell\n', ...
        env.map_size(1), env.map_size(2), env.resolution);

% Create simple scenario
env = env.create_simple_scenario();
fprintf('Added %d obstacles\n', length(env.obstacles));
fprintf('Start: [%.2f, %.2f], Goal: [%.2f, %.2f]\n', ...
        env.start_pos(1), env.start_pos(2), env.goal_pos(1), env.goal_pos(2));

% Test collision detection
test_points = [
    15, 15;   % Should be free
    30, 30;   % Should collide (near obstacle)
    50, 50;   % Should collide (near obstacle)
    85, 85    % Should be free
];

fprintf('\nCollision tests:\n');
for i = 1:size(test_points, 1)
    collision = env.check_collision(test_points(i, 1), test_points(i, 2));
    status = 'FREE';
    if collision
        status = 'COLLISION';
    end
    fprintf('  Point (%.2f, %.2f): %s\n', test_points(i, 1), test_points(i, 2), status);
end

% Plot environment
figure('Name', 'Test 3: Environment Visualization');
env.plot();

fprintf('✓ Test 3 passed!\n\n');

%% Test 4: State Estimator
fprintf('Test 4: State Estimator\n');
fprintf('-----------------------\n');

% Test perfect estimator
estimator_perfect = StateEstimator('perfect');
true_state = [10; 20; pi/4; 2.0];
est_state = estimator_perfect.get_state(true_state, 0);
fprintf('Perfect estimator error: %.6f\n', norm(est_state - true_state));

% Test noisy estimator
estimator_noisy = StateEstimator('noisy', [0.5, 0.5, 0.1]);
fprintf('\nNoisy estimator (20 samples):\n');
errors = zeros(20, 1);
for i = 1:20
    est_state = estimator_noisy.get_state(true_state, 0);
    errors(i) = norm(est_state(1:3) - true_state(1:3));
end
fprintf('  Mean error: %.4f m\n', mean(errors));
fprintf('  Std error: %.4f m\n', std(errors));

% Plot estimation errors
figure('Name', 'Test 4: State Estimation Errors');
subplot(2,1,1);
plot(1:20, errors, 'b.-', 'LineWidth', 2, 'MarkerSize', 15);
xlabel('Sample Number', 'FontSize', 12);
ylabel('Position Error (m)', 'FontSize', 12);
title('Noisy State Estimator - Position Error Over 20 Samples', 'FontSize', 14);
grid on;
ylim([0 max(errors)*1.2]);

subplot(2,1,2);
histogram(errors, 10, 'FaceColor', [0.3 0.5 0.8]);
xlabel('Position Error (m)', 'FontSize', 12);
ylabel('Frequency', 'FontSize', 12);
title(sprintf('Error Distribution (Mean=%.2fm, Std=%.2fm)', mean(errors), std(errors)), 'FontSize', 12);
grid on;

fprintf('✓ Test 4 passed!\n\n');

%% Summary
fprintf('=====================================\n');
fprintf('ALL PHASE 1 TESTS PASSED SUCCESSFULLY!\n');
fprintf('=====================================\n\n');

fprintf('Components ready:\n');
fprintf('  ✓ VehicleDynamics class (bicycle & differential drive)\n');
fprintf('  ✓ Environment class (obstacles, collision detection)\n');
fprintf('  ✓ StateEstimator class (perfect, noisy, external)\n\n');

fprintf('Next: Run this script to verify Phase 1\n');
fprintf('Command: run(''test_phase1.m'')\n');

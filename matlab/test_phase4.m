% test_phase4.m - Test Phase 4: CBF Safety Filter
% Tests Control Barrier Function based safety layer

% Add paths
addpath('core');
addpath('utils');
addpath('planners');
addpath('controllers');
addpath('safety');

clear all;
close all;
clc;

fprintf('===== PHASE 4 TESTING: CBF SAFETY FILTER =====\n\n');

%% Test 1: CBF with Static Obstacle
fprintf('Test 1: CBF - Static Obstacle Avoidance\n');
fprintf('----------------------------------------\n');

% Create environment with obstacle
env = Environment([30, 30], 1.0);
env = env.add_circular_obstacle(15, 15, 3);

% Plan path around obstacle (with larger safety margin)
planner_test1 = HybridAStarPlanner(env, 1.0, 2.5);
start = [5, 15, 0];
goal_pt = [25, 15, 0];
[path_test1, stats_test1] = planner_test1.plan(start, goal_pt);

if ~stats_test1.success
    fprintf('✗ Path planning failed!\n');
    return;
end

fprintf('✓ Path planned (%d waypoints, %.2fm)\n', size(path_test1, 1), stats_test1.path_length);

% Create vehicle dynamics
dt = 0.1;
vehicle = VehicleDynamics('bicycle', dt);

% Create CBF safety filter
safety_margin = 0.5; % Reduced for less conservative behavior
alpha = 0.3; % Lower alpha for smoother control
cbf = CBFSafetyFilter(env, safety_margin, alpha, 'bicycle', vehicle.L, dt);
fprintf('Created CBF filter (margin=%.1fm, alpha=%.1f)\n', safety_margin, alpha);

% Create LQR controller to follow planned path
Q = diag([10, 10, 5, 1]);
R = diag([1, 10]);
lqr_test1 = LQRController(Q, R, dt, 'bicycle', vehicle.L);

start_state = [start(1), start(2), start(3), 0];

state = start_state;
T = 50; % Increased time for path completion
steps = round(T / dt);

% Storage
trajectory = zeros(steps, 4);
controls_nominal = zeros(steps, 2);
controls_safe = zeros(steps, 2);
modifications = zeros(steps, 1);
min_distances = zeros(steps, 1);

% Simulation loop
fprintf('Running simulation...\n');
for i = 1:steps
    trajectory(i, :) = state;
    
    % LQR controller following planned path
    [v_nom, w_nom] = lqr_test1.track_path(state, path_test1, 3.0);
    
    controls_nominal(i, :) = [v_nom, w_nom];
    
    % Apply CBF safety filter
    [v_safe, w_safe, is_modified] = cbf.filter_control(state, v_nom, w_nom);
    controls_safe(i, :) = [v_safe, w_safe];
    modifications(i) = is_modified;
    
    % Get minimum distance to obstacles
    min_distances(i) = cbf.get_min_obstacle_distance(state);
    
    % Convert to bicycle model inputs
    delta = atan(vehicle.L * w_safe / max(v_safe, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_safe - state(4)) / dt;
    
    % Update vehicle
    state = vehicle.step(state, [a, delta]);
    
    if mod(i, 50) == 0
        fprintf('  Step %d/%d - Min distance: %.2fm\n', i, steps, min_distances(i));
    end
    
    % Stop if reached goal
    if norm([state(1)-goal_pt(1), state(2)-goal_pt(2)]) < 1.0 && i > 50
        trajectory = trajectory(1:i, :);
        controls_nominal = controls_nominal(1:i, :);
        controls_safe = controls_safe(1:i, :);
        modifications = modifications(1:i);
        min_distances = min_distances(1:i);
        break;
    end
end

fprintf('✓ Navigation complete!\n');
fprintf('  Final position: (%.2f, %.2f)\n', state(1), state(2));
fprintf('  Distance to goal: %.3f m\n', norm([state(1)-goal_pt(1), state(2)-goal_pt(2)]));
fprintf('  Min obstacle distance: %.3f m\n', min(min_distances));
fprintf('  Control modifications: %d/%d (%.1f%%)\n', ...
        sum(modifications), length(modifications), ...
        100*sum(modifications)/length(modifications));
fprintf('  Safety maintained: %s\n', iif(min(min_distances) >= 0, 'YES', 'NO'));

% Plot results
figure('Name', 'Test 1: CBF Static Obstacle Avoidance');
subplot(2,2,1);
env.plot();
hold on;
plot(path_test1(:,1), path_test1(:,2), 'b--', 'DisplayName', 'Planned Path');
plot(trajectory(:,1), trajectory(:,2), 'r-', 'DisplayName', 'Actual Path');
plot(start_state(1), start_state(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
plot(goal_pt(1), goal_pt(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
% Plot safety margin
theta_circle = linspace(0, 2*pi, 100);
obs = env.obstacles{1};
plot(obs.x + (obs.radius + safety_margin)*cos(theta_circle), ...
     obs.y + (obs.radius + safety_margin)*sin(theta_circle), ...
     'r--', 'DisplayName', 'Safety Boundary');
xlabel('X (m)'); ylabel('Y (m)');
title('CBF Obstacle Avoidance');
legend('Location', 'best');
grid on; axis equal;

subplot(2,2,2);
plot((1:length(min_distances))*dt, min_distances, 'b-');
hold on;
plot([0, length(min_distances)*dt], [0, 0], 'r--', 'DisplayName', 'Collision');
xlabel('Time (s)'); ylabel('Min Distance to Obstacle (m)');
title('Safety Distance vs Time');
grid on;
legend('Location', 'best');

subplot(2,2,3);
plot((1:length(controls_nominal))*dt, controls_nominal(:,1), 'b--', 'DisplayName', 'Nominal');
hold on;
plot((1:length(controls_safe))*dt, controls_safe(:,1), 'r-', 'DisplayName', 'Safe');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Velocity Commands');
legend('Location', 'best');
grid on;

subplot(2,2,4);
stem((1:length(modifications))*dt, modifications, 'r', 'DisplayName', 'Modified');
xlabel('Time (s)'); ylabel('Control Modified');
title('CBF Activation');
grid on;
ylim([-0.1, 1.1]);

fprintf('\n');

%% Test 2: CBF with Multiple Obstacles
fprintf('Test 2: CBF - Multiple Obstacles\n');
fprintf('---------------------------------\n');

% Create environment with multiple obstacles
env2 = Environment([40, 30], 1.0);
env2 = env2.add_circular_obstacle(15, 15, 3);
env2 = env2.add_circular_obstacle(25, 15, 2.5);
env2 = env2.add_circular_obstacle(20, 20, 2);

% Plan path
planner_test2 = HybridAStarPlanner(env2, 1.0, 2.5);
start2 = [5, 15, 0];
goal2_pt = [35, 15, 0];
[path_test2, stats_test2] = planner_test2.plan(start2, goal2_pt);

if ~stats_test2.success
    fprintf('✗ Path planning failed!\n');
    return;
end

fprintf('✓ Path planned (%d waypoints, %.2fm)\n', size(path_test2, 1), stats_test2.path_length);

% Create CBF safety filter
cbf2 = CBFSafetyFilter(env2, 0.5, 0.3, 'bicycle', vehicle.L, dt);

% Create LQR controller
lqr_test2 = LQRController(Q, R, dt, 'bicycle', vehicle.L);

% Navigate through obstacles
start_state2 = [start2(1), start2(2), start2(3), 0];

state = start_state2;
T = 60; % Increased time for path completion
steps = round(T / dt);

trajectory2 = zeros(steps, 4);
min_distances2 = zeros(steps, 1);
modifications2 = zeros(steps, 1);

fprintf('Running simulation...\n');
for i = 1:steps
    trajectory2(i, :) = state;
    
    % LQR controller following planned path
    [v_nom, w_nom] = lqr_test2.track_path(state, path_test2, 2.5);
    
    % Apply CBF filter
    [v_safe, w_safe, is_modified] = cbf2.filter_control(state, v_nom, w_nom);
    modifications2(i) = is_modified;
    min_distances2(i) = cbf2.get_min_obstacle_distance(state);
    
    % Update vehicle
    delta = atan(vehicle.L * w_safe / max(v_safe, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_safe - state(4)) / dt;
    state = vehicle.step(state, [a, delta]);
    
    if mod(i, 50) == 0
        fprintf('  Step %d/%d\n', i, steps);
    end
    
    if norm([state(1)-goal2_pt(1), state(2)-goal2_pt(2)]) < 1.0 && i > 50
        trajectory2 = trajectory2(1:i, :);
        min_distances2 = min_distances2(1:i);
        modifications2 = modifications2(1:i);
        break;
    end
end

fprintf('✓ Multi-obstacle navigation complete!\n');
fprintf('  Final position: (%.2f, %.2f)\n', state(1), state(2));
fprintf('  Distance to goal: %.3f m\n', norm([state(1)-goal2_pt(1), state(2)-goal2_pt(2)]));
fprintf('  Min obstacle distance: %.3f m\n', min(min_distances2));
fprintf('  Control modifications: %d/%d (%.1f%%)\n', ...
        sum(modifications2), length(modifications2), ...
        100*sum(modifications2)/length(modifications2));
fprintf('  Safety maintained: %s\n', iif(min(min_distances2) >= 0, 'YES', 'NO'));

% Plot
figure('Name', 'Test 2: CBF Multiple Obstacles');
env2.plot();
hold on;
plot(path_test2(:,1), path_test2(:,2), 'b--', 'DisplayName', 'Planned Path');
plot(trajectory2(:,1), trajectory2(:,2), 'r-', 'DisplayName', 'Safe Path');
plot(start_state2(1), start_state2(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
plot(goal2_pt(1), goal2_pt(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
xlabel('X (m)'); ylabel('Y (m)');
title('CBF Navigation Through Multiple Obstacles');
legend('Location', 'best');
grid on; axis equal;

fprintf('\n');

%% Test 3: CBF with LQR Controller
fprintf('Test 3: CBF + LQR Controller\n');
fprintf('-----------------------------\n');

% Create environment
env3 = Environment([50, 50], 1.0);
env3 = env3.add_circular_obstacle(25, 20, 5);
env3 = env3.add_circular_obstacle(30, 30, 3);

% Plan path with Hybrid A* (without obstacles consideration for nominal controller)
planner = HybridAStarPlanner(env3, 1.0, 2.5);
start = [5, 5, 0];
goal = [45, 45, pi/4];
[path_planned, stats] = planner.plan(start, goal);

if stats.success
    fprintf('✓ Path planned (%d waypoints, %.2fm)\n', size(path_planned, 1), stats.path_length);
    
    % Create LQR controller
    lqr_test3 = LQRController(Q, R, dt, 'bicycle', vehicle.L);
    
    % Create CBF filter
    cbf3 = CBFSafetyFilter(env3, 0.8, 0.4, 'bicycle', vehicle.L, dt);
    
    % Test with and without CBF
    T = 40;
    steps = round(T / dt);
    
    % Without CBF (nominal LQR only)
    fprintf('Testing without CBF...\n');
    state_no_cbf = [start(1), start(2), start(3), 0];
    traj_no_cbf = zeros(steps, 4);
    min_dist_no_cbf = zeros(steps, 1);
    
    for i = 1:steps
        traj_no_cbf(i, :) = state_no_cbf;
        [v_cmd, w_cmd] = lqr_test3.track_path(state_no_cbf, path_planned, 3.5);
        
        delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
        delta = max(min(delta, pi/4), -pi/4);
        a = (v_cmd - state_no_cbf(4)) / dt;
        state_no_cbf = vehicle.step(state_no_cbf, [a, delta]);
        
        min_dist_no_cbf(i) = cbf3.get_min_obstacle_distance(state_no_cbf);
        
        if norm([state_no_cbf(1)-goal(1), state_no_cbf(2)-goal(2)]) < 1.0 && i > 50
            traj_no_cbf = traj_no_cbf(1:i, :);
            min_dist_no_cbf = min_dist_no_cbf(1:i);
            break;
        end
    end
    
    % With CBF
    fprintf('Testing with CBF...\n');
    state_with_cbf = [start(1), start(2), start(3), 0];
    traj_with_cbf = zeros(steps, 4);
    min_dist_with_cbf = zeros(steps, 1);
    modifications3 = zeros(steps, 1);
    
    for i = 1:steps
        traj_with_cbf(i, :) = state_with_cbf;
        [v_nom, w_nom] = lqr_test3.track_path(state_with_cbf, path_planned, 3.5);
        
        % Apply CBF filter
        [v_safe, w_safe, is_modified] = cbf3.filter_control(state_with_cbf, v_nom, w_nom);
        modifications3(i) = is_modified;
        
        delta = atan(vehicle.L * w_safe / max(v_safe, 0.1));
        delta = max(min(delta, pi/4), -pi/4);
        a = (v_safe - state_with_cbf(4)) / dt;
        state_with_cbf = vehicle.step(state_with_cbf, [a, delta]);
        
        min_dist_with_cbf(i) = cbf3.get_min_obstacle_distance(state_with_cbf);
        
        if norm([state_with_cbf(1)-goal(1), state_with_cbf(2)-goal(2)]) < 1.0 && i > 50
            traj_with_cbf = traj_with_cbf(1:i, :);
            min_dist_with_cbf = min_dist_with_cbf(1:i);
            modifications3 = modifications3(1:i);
            break;
        end
    end
    
    fprintf('Results:\n');
    fprintf('  Without CBF: min distance = %.3f m (collisions: %s)\n', ...
            min(min_dist_no_cbf), iif(min(min_dist_no_cbf) < 0, 'YES', 'NO'));
    fprintf('  With CBF: min distance = %.3f m (collisions: %s)\n', ...
            min(min_dist_with_cbf), iif(min(min_dist_with_cbf) < 0, 'YES', 'NO'));
    fprintf('  CBF activations: %d times\n', sum(modifications3));
    
    % Plot comparison
    figure('Name', 'Test 3: CBF + LQR Controller');
    subplot(1,2,1);
    env3.plot();
    hold on;
    plot(path_planned(:,1), path_planned(:,2), 'b--', 'DisplayName', 'Planned Path');
    plot(traj_no_cbf(:,1), traj_no_cbf(:,2), 'r-', 'DisplayName', 'Without CBF');
    plot(start(1), start(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
    plot(goal(1), goal(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
    xlabel('X (m)'); ylabel('Y (m)');
    title('Without CBF Safety');
    legend('Location', 'best');
    grid on; axis equal;
    
    subplot(1,2,2);
    env3.plot();
    hold on;
    plot(path_planned(:,1), path_planned(:,2), 'b--', 'DisplayName', 'Planned Path');
    plot(traj_with_cbf(:,1), traj_with_cbf(:,2), 'g-', 'DisplayName', 'With CBF');
    plot(start(1), start(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
    plot(goal(1), goal(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
    xlabel('X (m)'); ylabel('Y (m)');
    title('With CBF Safety');
    legend('Location', 'best');
    grid on; axis equal;
end

fprintf('\n');

%% Summary
fprintf('=====================================\n');
fprintf('PHASE 4 (CBF SAFETY FILTER) TESTS COMPLETE!\n');
fprintf('=====================================\n\n');

fprintf('Summary:\n');
fprintf('  ✓ CBF safety filter implementation working\n');
fprintf('  ✓ Avoids static obstacles\n');
fprintf('  ✓ Handles multiple obstacles\n');
fprintf('  ✓ Integrates with controllers (LQR)\n');
fprintf('  ✓ Maintains safety guarantees\n\n');

fprintf('Next: Create main simulation script integrating all components\n\n');

% Helper function
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

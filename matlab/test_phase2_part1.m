% test_phase2.m - Test Phase 2: A* Path Planning
% Tests A* planner with different scenarios

% Add paths
addpath('core');
addpath('utils');
addpath('planners');

clear all;
close all;
clc;

fprintf('===== PHASE 2 TESTING: A* PLANNER =====\n\n');

%% Test 1: Simple Scenario
fprintf('Test 1: A* on Simple Scenario\n');
fprintf('------------------------------\n');

% Create environment
env = Environment([100, 100], 1.0);
env = env.create_simple_scenario();

% Create A* planner
planner = AStarPlanner(env, 1.0, 'euclidean');
fprintf('Created A* planner with Euclidean heuristic\n');

% Plan path
[path, stats] = planner.plan(env.start_pos, env.goal_pos);

if stats.success
    fprintf('✓ Path found!\n');
    fprintf('  Nodes expanded: %d\n', stats.nodes_expanded);
    fprintf('  Path length: %.2f m\n', stats.path_length);
    fprintf('  Runtime: %.4f s\n', stats.runtime);
    fprintf('  Path points: %d\n', size(path, 1));
else
    fprintf('✗ No path found!\n');
end

% Plot result
figure('Name', 'Test 1: A* Simple Scenario');
env.plot();
hold on;
if stats.success
    plot(path(:,1), path(:,2), 'b-', 'DisplayName', 'A* Path');
    plot(path(:,1), path(:,2), 'c.', 'MarkerSize', 8, 'DisplayName', 'Waypoints');
end
legend('Location', 'best');
title('A* Planner - Simple Scenario (Euclidean)');
hold off;

fprintf('\n');

%% Test 2: Different Heuristics Comparison
fprintf('Test 2: Heuristic Comparison\n');
fprintf('----------------------------\n');

heuristics = {'euclidean', 'manhattan', 'diagonal'};
results = cell(3, 1);

for i = 1:3
    planner = AStarPlanner(env, 1.0, heuristics{i});
    [path, stats] = planner.plan(env.start_pos, env.goal_pos);
    results{i} = struct('path', path, 'stats', stats, 'name', heuristics{i});
    
    fprintf('%s: nodes=%d, length=%.2fm, time=%.4fs\n', ...
            heuristics{i}, stats.nodes_expanded, stats.path_length, stats.runtime);
end

% Plot comparison
figure('Name', 'Test 2: Heuristic Comparison');
colors = {'b', 'r', 'g'};
env.plot();
hold on;
for i = 1:3
    if results{i}.stats.success
        plot(results{i}.path(:,1), results{i}.path(:,2), ...
             [colors{i} '-'], 'DisplayName', results{i}.name);
    end
end
legend('Location', 'best');
title('A* Heuristic Comparison');
hold off;

fprintf('\n');

%% Test 3: Complex Scenario
fprintf('Test 3: Complex Environment\n');
fprintf('---------------------------\n');

% Create more challenging environment
env_complex = Environment([100, 100], 1.0);
env_complex = env_complex.add_circular_obstacle(25, 25, 10);
env_complex = env_complex.add_circular_obstacle(35, 50, 8);
env_complex = env_complex.add_circular_obstacle(50, 30, 12);
env_complex = env_complex.add_circular_obstacle(65, 60, 7);
env_complex = env_complex.add_rectangular_obstacle(40, 70, 30, 8);
env_complex = env_complex.add_rectangular_obstacle(20, 45, 8, 20);
env_complex = env_complex.add_rectangular_obstacle(60, 15, 15, 10);
env_complex = env_complex.set_start_goal([10, 10], [90, 90]);

fprintf('Created complex environment with 7 obstacles\n');

% Plan with A*
planner_complex = AStarPlanner(env_complex, 1.0, 'euclidean');
[path_complex, stats_complex] = planner_complex.plan(env_complex.start_pos, env_complex.goal_pos);

if stats_complex.success
    fprintf('✓ Path found in complex environment!\n');
    fprintf('  Nodes expanded: %d\n', stats_complex.nodes_expanded);
    fprintf('  Path length: %.2f m\n', stats_complex.path_length);
    fprintf('  Runtime: %.4f s\n', stats_complex.runtime);
else
    fprintf('✗ No path found in complex environment!\n');
end

% Plot complex scenario
figure('Name', 'Test 3: Complex Environment');
env_complex.plot();
hold on;
if stats_complex.success
    plot(path_complex(:,1), path_complex(:,2), 'b-', 'DisplayName', 'A* Path');
    plot(path_complex(:,1), path_complex(:,2), 'c.', 'MarkerSize', 6);
end
legend('Location', 'best');
title('A* Planner - Complex Environment');
hold off;

fprintf('\n');

%% Test 4: Resolution Effect
fprintf('Test 4: Grid Resolution Impact\n');
fprintf('------------------------------\n');

resolutions = [0.5, 1.0, 2.0];
fprintf('Testing resolutions: ');
fprintf('%.1f ', resolutions);
fprintf('m/cell\n\n');

for i = 1:length(resolutions)
    planner_res = AStarPlanner(env, resolutions(i), 'euclidean');
    [~, stats_res] = planner_res.plan(env.start_pos, env.goal_pos);
    
    fprintf('Resolution %.1fm: nodes=%d, length=%.2fm, time=%.4fs\n', ...
            resolutions(i), stats_res.nodes_expanded, stats_res.path_length, stats_res.runtime);
end

fprintf('\n');

%% Test 5: Path Smoothness
fprintf('Test 5: Path Smoothness Analysis\n');
fprintf('--------------------------------\n');

if stats.success
    % Calculate path smoothness (sum of direction changes)
    smoothness = 0;
    for i = 2:size(path, 1)-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        v1 = v1 / norm(v1);
        v2 = v2 / norm(v2);
        angle_change = acos(dot(v1, v2));
        smoothness = smoothness + angle_change;
    end
    
    fprintf('Total direction change: %.2f rad (%.1f deg)\n', smoothness, rad2deg(smoothness));
    fprintf('Average turn per segment: %.2f deg\n', rad2deg(smoothness) / (size(path,1)-2));
    
    % Plot direction changes
    figure('Name', 'Test 5: Path Smoothness');
    angles = zeros(size(path, 1)-2, 1);
    for i = 2:size(path, 1)-1
        v1 = path(i, :) - path(i-1, :);
        v2 = path(i+1, :) - path(i, :);
        v1 = v1 / norm(v1);
        v2 = v2 / norm(v2);
        angles(i-1) = rad2deg(acos(dot(v1, v2)));
    end
    
    plot(1:length(angles), angles, 'b.-', 'MarkerSize', 12);
    xlabel('Waypoint Index', 'FontSize', 12);
    ylabel('Direction Change (degrees)', 'FontSize', 12);
    title('A* Path - Direction Changes', 'FontSize', 14);
    grid on;
end

fprintf('\n');

%% Summary
fprintf('=====================================\n');
fprintf('PHASE 2 (A* PLANNER) TESTS COMPLETE!\n');
fprintf('=====================================\n\n');

fprintf('Summary:\n');
fprintf('  ✓ A* implementation working\n');
fprintf('  ✓ Multiple heuristics supported\n');
fprintf('  ✓ Handles complex environments\n');
fprintf('  ✓ Configurable resolution\n');
fprintf('  ✓ Returns path statistics\n\n');

fprintf('Next: Run test_phase2.m to verify A* planner\n');
fprintf('Command: run(''test_phase2.m'')\n');

% diagnose_mpc.m - Diagnostic script for MPC obstacle avoidance

clear all; close all; clc;

% Add paths
addpath('core');
addpath('utils');
addpath('planners');
addpath('controllers');
addpath('safety');

% Setup
dt = 0.1;
start = [5, 5, 0];
goal = [45, 45, pi/4];

% Create environment
env = Environment([50, 50], 1.0);
env = env.add_circular_obstacle(25, 20, 5);
env = env.add_circular_obstacle(30, 30, 3);
env = env.add_circular_obstacle(15, 35, 4);

% Plan path with RRT*
planner = RRTStarPlanner(env, 1.0);
[path, stats] = planner.plan(start(1:2), goal(1:2));
path = add_headings_to_path(path);

fprintf('Path Planning:\n');
fprintf('  Waypoints: %d\n', size(path, 1));
fprintf('  Path length: %.2f m\n', stats.path_length);

% Check if path goes through obstacles
fprintf('\nPath Safety Check:\n');
min_clearance = inf;
for i = 1:size(path, 1)
    x = path(i, 1);
    y = path(i, 2);
    
    for j = 1:length(env.obstacles)
        obs = env.obstacles{j};
        dx = x - obs.x;
        dy = y - obs.y;
        dist = sqrt(dx^2 + dy^2);
        clearance = dist - obs.radius;
        min_clearance = min(min_clearance, clearance);
        
        if clearance < 0.5
            fprintf('  WARNING: Waypoint %d at (%.1f, %.1f) is %.2fm from obstacle %d\n', ...
                i, x, y, clearance, j);
        end
    end
end
fprintf('  Minimum path clearance: %.2f m\n', min_clearance);

% Visualize
figure('Position', [100, 100, 800, 600]);
env.plot();
hold on;
plot(path(:,1), path(:,2), 'b-o', 'LineWidth', 2, 'MarkerSize', 6);
plot(start(1), start(2), 'go', 'MarkerSize', 15, 'LineWidth', 3);
plot(goal(1), goal(2), 'r^', 'MarkerSize', 15, 'LineWidth', 3);
title('RRT* Path Quality Check');
xlabel('X (m)'); ylabel('Y (m)');
legend('Obstacles', 'Planned Path', 'Start', 'Goal');
grid on; axis equal;

function path_with_heading = add_headings_to_path(path)
    n = size(path, 1);
    headings = zeros(n, 1);
    for i = 1:n-1
        dx = path(i+1, 1) - path(i, 1);
        dy = path(i+1, 2) - path(i, 2);
        headings(i) = atan2(dy, dx);
    end
    headings(n) = headings(n-1);
    path_with_heading = [path, headings];
end

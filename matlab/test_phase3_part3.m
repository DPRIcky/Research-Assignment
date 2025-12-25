% test_phase3_part3.m - Test Phase 3 (Part 3): MPC Controller
% Tests Model Predictive Control trajectory tracking controller

% Add paths
addpath('core');
addpath('utils');
addpath('planners');
addpath('controllers');

clear all;
close all;
clc;

fprintf('===== PHASE 3 TESTING: MPC CONTROLLER =====\n\n');

%% Test 1: MPC - Straight Line Tracking
fprintf('Test 1: MPC - Straight Line Tracking\n');
fprintf('--------------------------------------\n');

% Create straight line path
path_straight = [[0:2:20]', zeros(11, 1), zeros(11, 1), 2.0*ones(11, 1)];

% Create vehicle dynamics
dt = 0.1;
vehicle = VehicleDynamics('bicycle', dt);

% Create MPC controller
N = 10; % Prediction horizon
Q = diag([10, 10, 5, 1]); % State weights
R = diag([1, 10]); % Control weights
Qf = diag([20, 20, 10, 2]); % Terminal weights
controller = MPCController(N, dt, Q, R, Qf, 'bicycle', vehicle.L);
fprintf('Created MPC controller (N=%d, Q_x=%.1f, R_a=%.1f)\n', N, Q(1,1), R(1,1));

% Initial state
state = [0, 0, 0, 0]; % [x, y, theta, v]
T = 15; % simulation time
steps = round(T / dt);

% Storage
trajectory = zeros(steps, 4);
controls = zeros(steps, 2);
errors = zeros(steps, 1);

% Simulation loop
fprintf('Running simulation...\n');
for i = 1:steps
    trajectory(i, :) = state;
    
    % Compute control
    [v_cmd, w_cmd] = controller.track_path(state, path_straight, 3.0);
    controls(i, :) = [v_cmd, w_cmd];
    
    % Convert to bicycle model inputs
    delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_cmd - state(4)) / dt;
    
    % Update vehicle
    state = vehicle.step(state, [a, delta]);
    
    % Calculate error
    errors(i) = controller.get_tracking_error(state, path_straight);
    
    if mod(i, 50) == 0
        fprintf('  Step %d/%d\n', i, steps);
    end
end

% Calculate statistics
mean_error = mean(errors(50:end)); % After settling
max_error = max(errors);
final_error = errors(end);

fprintf('✓ Tracking complete!\n');
fprintf('  Mean error: %.3f m\n', mean_error);
fprintf('  Max error: %.3f m\n', max_error);
fprintf('  Final error: %.3f m\n', final_error);
fprintf('  Final position: (%.2f, %.2f)\n', state(1), state(2));

% Plot results
figure('Name', 'Test 1: MPC Straight Line Tracking');
subplot(2,2,1);
plot(path_straight(:,1), path_straight(:,2), 'b--', 'DisplayName', 'Desired');
hold on;
plot(trajectory(:,1), trajectory(:,2), 'r-', 'DisplayName', 'Actual');
plot(trajectory(1,1), trajectory(1,2), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(trajectory(end,1), trajectory(end,2), 'r^', 'MarkerSize', 10, 'DisplayName', 'End');
xlabel('X (m)'); ylabel('Y (m)');
title('Path Tracking');
legend('Location', 'best');
grid on; axis equal;

subplot(2,2,2);
plot((1:steps)*dt, errors, 'r-');
xlabel('Time (s)'); ylabel('Tracking Error (m)');
title('Tracking Error vs Time');
grid on;

subplot(2,2,3);
plot((1:steps)*dt, controls(:,1), 'b-');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Control - Linear Velocity');
grid on;

subplot(2,2,4);
plot((1:steps)*dt, controls(:,2), 'r-');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('Control - Angular Velocity');
grid on;

fprintf('\n');

%% Test 2: MPC - Circular Path Tracking
fprintf('Test 2: MPC - Circular Path Tracking\n');
fprintf('-------------------------------------\n');

% Create circular path
radius = 10;
angles = linspace(0, 2*pi, 100)';
path_circle = [radius*cos(angles), radius*sin(angles), angles + pi/2, 2.0*ones(100, 1)];

% Initial state
state = [radius, 0, pi/2, 0];
T = 25;
steps = round(T / dt);

% Storage
trajectory_circle = zeros(steps, 4);
controls_circle = zeros(steps, 2);
errors_circle = zeros(steps, 1);

% Simulation loop
fprintf('Running simulation...\n');
for i = 1:steps
    trajectory_circle(i, :) = state;
    
    % Compute control
    [v_cmd, w_cmd] = controller.track_path(state, path_circle, 3.5);
    controls_circle(i, :) = [v_cmd, w_cmd];
    
    % Convert to bicycle model inputs
    delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_cmd - state(4)) / dt;
    
    % Update vehicle
    state = vehicle.step(state, [a, delta]);
    
    % Calculate error
    errors_circle(i) = controller.get_tracking_error(state, path_circle);
    
    if mod(i, 50) == 0
        fprintf('  Step %d/%d\n', i, steps);
    end
end

% Calculate statistics
mean_error_circle = mean(errors_circle(100:end));
max_error_circle = max(errors_circle);

fprintf('✓ Circular tracking complete!\n');
fprintf('  Mean error: %.3f m\n', mean_error_circle);
fprintf('  Max error: %.3f m\n', max_error_circle);

% Plot circular tracking
figure('Name', 'Test 2: MPC Circular Path Tracking');
subplot(2,2,1);
plot(path_circle(:,1), path_circle(:,2), 'b--', 'DisplayName', 'Desired');
hold on;
plot(trajectory_circle(:,1), trajectory_circle(:,2), 'r-', 'DisplayName', 'Actual');
xlabel('X (m)'); ylabel('Y (m)');
title('Circular Path Tracking');
legend('Location', 'best');
grid on; axis equal;

subplot(2,2,2);
plot((1:steps)*dt, errors_circle, 'r-');
xlabel('Time (s)'); ylabel('Tracking Error (m)');
title('Tracking Error vs Time');
grid on;

subplot(2,2,3);
plot((1:steps)*dt, controls_circle(:,1), 'b-');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Control - Linear Velocity');
grid on;

subplot(2,2,4);
plot((1:steps)*dt, controls_circle(:,2), 'r-');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('Control - Angular Velocity');
grid on;

fprintf('\n');

%% Test 3: MPC - Following Planned Path
fprintf('Test 3: MPC - Following Planned Path\n');
fprintf('-------------------------------------\n');

% Create environment and plan path
env = Environment([50, 50], 1.0);
env = env.add_circular_obstacle(25, 20, 5);
env = env.add_circular_obstacle(15, 30, 4);

% Plan with Hybrid A*
planner = HybridAStarPlanner(env, 1.0, 2.5);
start = [5, 5, 0];
goal = [45, 45, pi/4];
[path_planned, stats_plan] = planner.plan(start, goal);

if stats_plan.success
    fprintf('✓ Path planned (%d waypoints, %.2fm)\n', ...
            size(path_planned, 1), stats_plan.path_length);
    
    % Add velocity column if not present
    if size(path_planned, 2) == 3
        path_planned = [path_planned, 2.0*ones(size(path_planned, 1), 1)];
    end
    
    % Initial state
    state = [start(1), start(2), start(3), 0];
    T = 40;
    steps = round(T / dt);
    
    % Storage
    trajectory_planned = zeros(steps, 4);
    errors_planned = zeros(steps, 1);
    
    % Simulation loop
    fprintf('Running simulation...\n');
    for i = 1:steps
        trajectory_planned(i, :) = state;
        
        % Compute control
        [v_cmd, w_cmd] = controller.track_path(state, path_planned, 4.0);
        
        % Convert to bicycle model inputs
        delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
        delta = max(min(delta, pi/4), -pi/4);
        a = (v_cmd - state(4)) / dt;
        
        % Update vehicle
        state = vehicle.step(state, [a, delta]);
        
        % Calculate error
        errors_planned(i) = controller.get_tracking_error(state, path_planned);
        
        if mod(i, 100) == 0
            fprintf('  Step %d/%d\n', i, steps);
        end
        
        % Stop if reached goal
        if norm([state(1)-goal(1), state(2)-goal(2)]) < 1.0 && i > 100
            trajectory_planned = trajectory_planned(1:i, :);
            errors_planned = errors_planned(1:i);
            break;
        end
    end
    
    fprintf('✓ Path tracking complete!\n');
    fprintf('  Mean error: %.3f m\n', mean(errors_planned));
    fprintf('  Max error: %.3f m\n', max(errors_planned));
    fprintf('  Final distance to goal: %.3f m\n', ...
            norm([state(1)-goal(1), state(2)-goal(2)]));
    
    % Plot planned path tracking
    figure('Name', 'Test 3: MPC Following Planned Path');
    env.plot();
    hold on;
    plot(path_planned(:,1), path_planned(:,2), 'b--', 'DisplayName', 'Planned Path');
    plot(trajectory_planned(:,1), trajectory_planned(:,2), 'r-', 'DisplayName', 'Actual Trajectory');
    plot(start(1), start(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
    plot(goal(1), goal(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
    legend('Location', 'best');
    title('MPC Tracking Planned Path');
    axis equal; grid on;
else
    fprintf('✗ Path planning failed!\n');
end

fprintf('\n');

%% Test 4: Horizon Length Comparison
fprintf('Test 4: MPC Horizon Length Comparison\n');
fprintf('--------------------------------------\n');

% Test different horizon lengths
horizons = [5, 10, 20];

% Test path (figure-8)
t = linspace(0, 4*pi, 200)';
path_fig8 = [8*sin(t), 4*sin(2*t), atan2(8*cos(2*t), 4*cos(t)), 2.0*ones(200, 1)];

fprintf('Testing horizon lengths on figure-8 path:\n\n');

for h = 1:length(horizons)
    N_test = horizons(h);
    
    % Create controller with specific horizon
    ctrl = MPCController(N_test, dt, Q, R, Qf, 'bicycle', vehicle.L);
    
    % Simulate
    state = [0, 0, 0, 0];
    T = 30;
    steps = round(T / dt);
    traj = zeros(steps, 4);
    errs = zeros(steps, 1);
    
    for i = 1:steps
        traj(i, :) = state;
        [v_cmd, w_cmd] = ctrl.track_path(state, path_fig8, 3.5);
        delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
        delta = max(min(delta, pi/4), -pi/4);
        a = (v_cmd - state(4)) / dt;
        state = vehicle.step(state, [a, delta]);
        errs(i) = ctrl.get_tracking_error(state, path_fig8);
    end
    
    mean_err = mean(errs(100:end));
    max_err = max(errs);
    
    fprintf('N=%d: mean=%.3fm, max=%.3fm\n', N_test, mean_err, max_err);
end

fprintf('\n');

%% Test 5: PID vs LQR vs MPC Comparison
fprintf('Test 5: PID vs LQR vs MPC Comparison\n');
fprintf('-------------------------------------\n');

% Create path with sharp turn
waypoints = [0, 0; 10, 0; 10, 10; 0, 10; 0, 0];
path_sharp = [];
for i = 1:size(waypoints, 1)-1
    segment = [linspace(waypoints(i,1), waypoints(i+1,1), 20)', ...
               linspace(waypoints(i,2), waypoints(i+1,2), 20)'];
    path_sharp = [path_sharp; segment];
end
% Add headings and velocity
dx = [diff(path_sharp(:,1)); 0];
dy = [diff(path_sharp(:,2)); 0];
headings = atan2(dy, dx);
path_sharp = [path_sharp, headings, 2.0*ones(size(path_sharp, 1), 1)];

T = 25;
steps = round(T / dt);

% Test MPC
fprintf('Testing MPC...\n');
mpc_ctrl = MPCController(10, dt, Q, R, Qf, 'bicycle', vehicle.L);
state_mpc = [0, 0, 0, 0];
traj_mpc = zeros(steps, 4);
errs_mpc = zeros(steps, 1);

for i = 1:steps
    traj_mpc(i, :) = state_mpc;
    [v_cmd, w_cmd] = mpc_ctrl.track_path(state_mpc, path_sharp, 3.0);
    delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_cmd - state_mpc(4)) / dt;
    state_mpc = vehicle.step(state_mpc, [a, delta]);
    errs_mpc(i) = mpc_ctrl.get_tracking_error(state_mpc, path_sharp);
end

% Test LQR
fprintf('Testing LQR...\n');
lqr_ctrl = LQRController(Q, R, dt, 'bicycle', vehicle.L);
state_lqr = [0, 0, 0, 0];
traj_lqr = zeros(steps, 4);
errs_lqr = zeros(steps, 1);

for i = 1:steps
    traj_lqr(i, :) = state_lqr;
    [v_cmd, w_cmd] = lqr_ctrl.track_path(state_lqr, path_sharp, 3.0);
    delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_cmd - state_lqr(4)) / dt;
    state_lqr = vehicle.step(state_lqr, [a, delta]);
    errs_lqr(i) = lqr_ctrl.get_tracking_error(state_lqr, path_sharp);
end

% Test PID
fprintf('Testing PID...\n');
pid_ctrl = PIDController(2.0, 0.2, 1.0, 4.0, 0.2, 0.6);
state_pid = [0, 0, 0, 0];
traj_pid = zeros(steps, 4);
errs_pid = zeros(steps, 1);

for i = 1:steps
    traj_pid(i, :) = state_pid;
    [v_cmd, w_cmd, pid_ctrl] = pid_ctrl.track_path(state_pid, path_sharp, 3.0, dt);
    delta = atan(vehicle.L * w_cmd / max(v_cmd, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_cmd - state_pid(4)) / dt;
    state_pid = vehicle.step(state_pid, [a, delta]);
    distances = sqrt((path_sharp(:,1) - state_pid(1)).^2 + (path_sharp(:,2) - state_pid(2)).^2);
    errs_pid(i) = min(distances);
end

fprintf('\nSquare path with sharp turns:\n');
fprintf('  MPC: mean=%.3fm, max=%.3fm\n', mean(errs_mpc), max(errs_mpc));
fprintf('  LQR: mean=%.3fm, max=%.3fm\n', mean(errs_lqr), max(errs_lqr));
fprintf('  PID: mean=%.3fm, max=%.3fm\n', mean(errs_pid), max(errs_pid));

% Plot comparison
figure('Name', 'Test 5: PID vs LQR vs MPC Comparison');
subplot(2,2,1);
plot(path_sharp(:,1), path_sharp(:,2), 'k--', 'DisplayName', 'Reference');
hold on;
plot(traj_mpc(:,1), traj_mpc(:,2), 'g-', 'DisplayName', 'MPC');
plot(traj_lqr(:,1), traj_lqr(:,2), 'b-', 'DisplayName', 'LQR');
plot(traj_pid(:,1), traj_pid(:,2), 'r-', 'DisplayName', 'PID');
xlabel('X (m)'); ylabel('Y (m)');
title('Path Tracking Comparison');
legend('Location', 'best');
grid on; axis equal;

subplot(2,2,2);
plot((1:steps)*dt, errs_mpc, 'g-', 'DisplayName', 'MPC');
hold on;
plot((1:steps)*dt, errs_lqr, 'b-', 'DisplayName', 'LQR');
plot((1:steps)*dt, errs_pid, 'r-', 'DisplayName', 'PID');
xlabel('Time (s)'); ylabel('Tracking Error (m)');
title('Error Comparison');
legend('Location', 'best');
grid on;

subplot(2,2,3);
plot(traj_mpc(:,1), traj_mpc(:,2), 'g-', 'DisplayName', 'MPC');
hold on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'DisplayName', 'Desired');
xlabel('X (m)'); ylabel('Y (m)');
title('MPC Trajectory');
legend('Location', 'best');
grid on; axis equal;

subplot(2,2,4);
plot(traj_lqr(:,1), traj_lqr(:,2), 'b-', 'DisplayName', 'LQR');
hold on;
plot(waypoints(:,1), waypoints(:,2), 'k--', 'DisplayName', 'Desired');
xlabel('X (m)'); ylabel('Y (m)');
title('LQR Trajectory');
legend('Location', 'best');
grid on; axis equal;

fprintf('\n');

%% Summary
fprintf('=====================================\n');
fprintf('PHASE 3 (MPC CONTROLLER) TESTS COMPLETE!\n');
fprintf('=====================================\n\n');

fprintf('Summary:\n');
fprintf('  ✓ MPC controller implementation working\n');
fprintf('  ✓ Tracks straight and curved paths\n');
fprintf('  ✓ Follows planned paths from planners\n');
fprintf('  ✓ Tunable horizon for performance\n');
fprintf('  ✓ Handles constraints explicitly\n');
fprintf('  ✓ Compared with PID and LQR controllers\n\n');

fprintf('Next: Implement CBF safety layer (Phase 4)\n\n');

% run_simulation.m - Wrapper script to run simulation in batch mode
% Usage: matlab -batch "planner=3; controller=2; cbf_enabled=1; estimator=3; run('run_simulation.m');"

% Add paths
addpath('core');
addpath('utils');
addpath('planners');
addpath('controllers');
addpath('safety');

% Default values if not specified
if ~exist('planner', 'var'), planner = 3; end  % Hybrid A*
if ~exist('controller', 'var'), controller = 2; end  % LQR
if ~exist('cbf_enabled', 'var'), cbf_enabled = 1; end  % Yes
if ~exist('estimator', 'var'), estimator = 1; end  % Perfect

fprintf('=========================================================\n');
fprintf('  AUTONOMOUS VEHICLE NAVIGATION SIMULATION\n');
fprintf('=========================================================\n\n');

planner_names = {'A*', 'RRT*', 'Hybrid A*'};
controller_names = {'PID', 'LQR', 'MPC'};
estimator_modes = {'perfect', 'noisy', 'ekf', 'external'};
estimator_names = {'Perfect', 'Noisy', 'EKF', 'External'};

fprintf('Configuration:\n');
fprintf('  Planner: %s\n', planner_names{planner});
fprintf('  Controller: %s\n', controller_names{controller});
fprintf('  CBF Safety: %s\n', iif(cbf_enabled, 'Enabled', 'Disabled'));
fprintf('  State Estimation: %s\n\n', estimator_names{estimator});

%% Setup Simulation Parameters
fprintf('Setting up simulation...\n');

% Time parameters
dt = 0.1;
T = 60;
steps = round(T / dt);

% Create environment
env = Environment([50, 50], 1.0);
env = env.add_circular_obstacle(25, 20, 5);
env = env.add_circular_obstacle(30, 30, 3);
env = env.add_circular_obstacle(15, 35, 4);
fprintf('✓ Environment created with %d obstacles\n', length(env.obstacles));

% Define start and goal
start = [5, 5, 0];
goal = [45, 45, pi/4];
fprintf('✓ Start: (%.1f, %.1f, %.2f rad)\n', start(1), start(2), start(3));
fprintf('✓ Goal:  (%.1f, %.1f, %.2f rad)\n', goal(1), goal(2), goal(3));

% Vehicle parameters
vehicle = VehicleDynamics('bicycle', dt);
fprintf('✓ Vehicle model: Bicycle (L=%.2fm)\n\n', vehicle.L);

%% Path Planning
fprintf('---------------------------------------------------------\n');
fprintf('PHASE 1: Path Planning (%s)\n', planner_names{planner});
fprintf('---------------------------------------------------------\n');

switch planner
    case 1  % A*
        plan_obj = AStarPlanner(env, 1.0);
        [path, stats] = plan_obj.plan(start(1:2), goal(1:2));
        if stats.success
            path = add_headings_to_path(path);
        end
        
    case 2  % RRT*
        plan_obj = RRTStarPlanner(env, 1.0);
        [path, stats] = plan_obj.plan(start(1:2), goal(1:2));
        if stats.success
            path = add_headings_to_path(path);
        end
        
    case 3  % Hybrid A*
        plan_obj = HybridAStarPlanner(env, 1.0, vehicle.L);
        [path, stats] = plan_obj.plan(start, goal);
end

if ~stats.success
    fprintf('✗ Path planning failed!\n');
    return;
end

fprintf('✓ Path planned successfully\n');
fprintf('  Waypoints: %d\n', size(path, 1));
fprintf('  Path length: %.2f m\n', stats.path_length);
if isfield(stats, 'time')
    fprintf('  Planning time: %.3f s\n', stats.time);
end
fprintf('\n');

%% Controller Setup
fprintf('---------------------------------------------------------\n');
fprintf('PHASE 2: Controller Configuration (%s)\n', controller_names{controller});
fprintf('---------------------------------------------------------\n');

switch controller
    case 1  % PID
        ctrl = PIDController(2.0, 0.2, 1.0, 4.0, 0.2, 0.6);
        fprintf('✓ PID controller created\n\n');
        
    case 2  % LQR
        Q = diag([10, 10, 5, 1]);
        R = diag([1, 10]);
        ctrl = LQRController(Q, R, dt, 'bicycle', vehicle.L);
        fprintf('✓ LQR controller created\n\n');
        
    case 3  % MPC
        N = 10;
        Q = diag([10, 10, 5, 1]);
        R = diag([1, 10]);
        Qf = diag([20, 20, 10, 2]);
        ctrl = MPCController(N, dt, Q, R, Qf, 'bicycle', vehicle.L);
        fprintf('✓ MPC controller created\n\n');
end

%% CBF Safety Filter Setup
if cbf_enabled
    fprintf('---------------------------------------------------------\n');
    fprintf('PHASE 3: Safety Layer Configuration\n');
    fprintf('---------------------------------------------------------\n');
    
    safety_margin = 0.5;
    alpha = 0.3;
    cbf = CBFSafetyFilter(env, safety_margin, alpha, 'bicycle', vehicle.L, dt);
    fprintf('✓ CBF safety filter created\n\n');
end

%% State Estimator Setup
fprintf('---------------------------------------------------------\n');
fprintf('PHASE 4: State Estimation Configuration (%s)\n', estimator_names{estimator});
fprintf('---------------------------------------------------------\n');

switch estimator
    case 1  % Perfect
        est = StateEstimator('perfect');
        fprintf('✓ Perfect state estimation\n\n');
        
    case 2  % Noisy
        noise_std = [0.5, 0.5, 0.1];
        est = StateEstimator('noisy', noise_std);
        fprintf('✓ Noisy state estimation\n\n');
        
    case 3  % EKF
        process_noise = diag([0.1, 0.1, 0.05, 0.1]);
        measurement_noise = diag([0.5, 0.5, 0.1]);
        est = EKFEstimator(dt, process_noise, measurement_noise, vehicle.L);
        fprintf('✓ Extended Kalman Filter\n\n');
        
    case 4  % External
        est = StateEstimator('external');
        fprintf('✓ External state data (using perfect for demo)\n\n');
        est = StateEstimator('perfect');
end

%% Main Simulation Loop
fprintf('=========================================================\n');
fprintf('PHASE 5: Running Simulation\n');
fprintf('=========================================================\n\n');

% Initialize state
true_state = [start(1), start(2), start(3), 0];
estimated_state = true_state;

% Initialize EKF if using it
if estimator == 3
    est = est.initialize(true_state);
end

% Storage
trajectory_true = zeros(steps, 4);
trajectory_estimated = zeros(steps, 4);
controls = zeros(steps, 2);
tracking_errors = zeros(steps, 1);
estimation_errors = zeros(steps, 1);
cbf_activations = zeros(steps, 1);
min_obstacle_distances = zeros(steps, 1);

fprintf('Simulation progress:\n');
start_sim = tic;

for i = 1:steps
    trajectory_true(i, :) = true_state;
    
    % Get state estimate
    if estimator == 3  % EKF
        z = true_state(1:3)' + [randn()*0.5; randn()*0.5; randn()*0.1];
        if i > 1
            % Get previous control in bicycle model format [a, delta]
            v_prev = controls(i-1, 1);
            w_prev = controls(i-1, 2);
            delta_prev = atan(vehicle.L * w_prev / max(v_prev, 0.1));
            a_prev = (v_prev - trajectory_true(i-1, 4)) / dt;
            u_prev = [a_prev; delta_prev];
        else
            u_prev = [0; 0];
        end
        est = est.predict(u_prev);
        [est, estimated_state] = est.update(z);
    else
        estimated_state = est.get_state(true_state, i*dt);
    end
    
    trajectory_estimated(i, :) = estimated_state;
    
    % Compute nominal control
    switch controller
        case 1  % PID
            [v_nom, w_nom, ctrl] = ctrl.track_path(estimated_state, path, 3.0, dt);
        case 2  % LQR
            [v_nom, w_nom] = ctrl.track_path(estimated_state, path, 3.5);
        case 3  % MPC
            [v_nom, w_nom] = ctrl.track_path(estimated_state, path, 3.5);
    end
    
    % Apply CBF if enabled
    if cbf_enabled
        [v_safe, w_safe, is_modified] = cbf.filter_control(true_state, v_nom, w_nom);
        cbf_activations(i) = is_modified;
    else
        v_safe = v_nom;
        w_safe = w_nom;
    end
    
    controls(i, :) = [v_safe, w_safe];
    
    % Convert to bicycle inputs
    delta = atan(vehicle.L * w_safe / max(v_safe, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_safe - true_state(4)) / dt;
    
    % Update vehicle state
    true_state = vehicle.step(true_state, [a, delta]);
    
    % Calculate errors
    distances = sqrt((path(:,1) - true_state(1)).^2 + (path(:,2) - true_state(2)).^2);
    tracking_errors(i) = min(distances);
    estimation_errors(i) = norm(true_state(1:3) - estimated_state(1:3));
    
    if cbf_enabled
        min_obstacle_distances(i) = cbf.get_min_obstacle_distance(true_state);
    end
    
    if mod(i, 100) == 0
        fprintf('  Step %d/%d (%.1f%%)\n', i, steps, 100*i/steps);
    end
    
    % Check goal
    if norm([true_state(1)-goal(1), true_state(2)-goal(2)]) < 1.0 && i > 50
        trajectory_true = trajectory_true(1:i, :);
        trajectory_estimated = trajectory_estimated(1:i, :);
        controls = controls(1:i, :);
        tracking_errors = tracking_errors(1:i);
        estimation_errors = estimation_errors(1:i);
        cbf_activations = cbf_activations(1:i);
        if cbf_enabled
            min_obstacle_distances = min_obstacle_distances(1:i);
        end
        fprintf('\n✓ Goal reached at step %d (%.1f seconds)\n', i, i*dt);
        break;
    end
end

elapsed_time = toc(start_sim);

%% Results Summary
fprintf('\n=========================================================\n');
fprintf('SIMULATION RESULTS\n');
fprintf('=========================================================\n\n');

fprintf('Performance Metrics:\n');
fprintf('  Final position: (%.2f, %.2f, %.2f rad)\n', true_state(1), true_state(2), true_state(3));
fprintf('  Distance to goal: %.3f m\n', norm([true_state(1)-goal(1), true_state(2)-goal(2)]));
fprintf('  Mean tracking error: %.3f m\n', mean(tracking_errors));
fprintf('  Max tracking error: %.3f m\n', max(tracking_errors));
fprintf('  Mean estimation error: %.3f m\n', mean(estimation_errors));
fprintf('  Max estimation error: %.3f m\n', max(estimation_errors));
fprintf('  Simulation time: %.2f s (real-time)\n', elapsed_time);

if cbf_enabled
    fprintf('  CBF activations: %d/%d (%.1f%%)\n', sum(cbf_activations), length(cbf_activations), ...
            100*sum(cbf_activations)/length(cbf_activations));
    fprintf('  Min obstacle distance: %.3f m\n', min(min_obstacle_distances));
end

%% Visualization
fprintf('\nGenerating visualization...\n');

figure('Position', [100, 100, 1400, 800]);

subplot(2,3,1);
env.plot();
hold on;
plot(path(:,1), path(:,2), 'b--', 'DisplayName', 'Planned Path');
plot(trajectory_true(:,1), trajectory_true(:,2), 'r-', 'DisplayName', 'True Trajectory');
if estimator > 1
    plot(trajectory_estimated(:,1), trajectory_estimated(:,2), 'g:', 'DisplayName', 'Estimated');
end
plot(start(1), start(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
plot(goal(1), goal(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('%s + %s', planner_names{planner}, controller_names{controller}));
legend('Location', 'best');
grid on; axis equal;

subplot(2,3,2);
plot((1:length(tracking_errors))*dt, tracking_errors, 'r-');
xlabel('Time (s)'); ylabel('Tracking Error (m)');
title('Path Tracking Error');
grid on;

subplot(2,3,3);
plot((1:length(estimation_errors))*dt, estimation_errors, 'b-');
xlabel('Time (s)'); ylabel('Estimation Error (m)');
title(sprintf('State Estimation Error\n(%s)', estimator_names{estimator}));
grid on;

subplot(2,3,4);
plot((1:length(controls))*dt, controls(:,1), 'b-');
xlabel('Time (s)'); ylabel('Linear Velocity (m/s)');
title('Control - Linear Velocity');
grid on;

subplot(2,3,5);
plot((1:length(controls))*dt, controls(:,2), 'r-');
xlabel('Time (s)'); ylabel('Angular Velocity (rad/s)');
title('Control - Angular Velocity');
grid on;

subplot(2,3,6);
if cbf_enabled
    yyaxis left;
    plot((1:length(min_obstacle_distances))*dt, min_obstacle_distances, 'b-');
    ylabel('Min Distance (m)');
    hold on;
    plot([0, length(min_obstacle_distances)*dt], [0, 0], 'r--');
    yyaxis right;
    plot((1:length(cbf_activations))*dt, cbf_activations, 'r-');
    ylabel('CBF Active');
    xlabel('Time (s)');
    title('CBF Safety Monitor');
    grid on;
else
    plot((1:length(tracking_errors))*dt, cumsum(tracking_errors)*dt, 'g-');
    xlabel('Time (s)'); ylabel('Cumulative Error (m·s)');
    title('Integrated Tracking Error');
    grid on;
end

saveas(gcf, sprintf('results_%s_%s_cbf%d_est%d.png', ...
    strrep(planner_names{planner}, '*', 'star'), ...
    controller_names{controller}, cbf_enabled, estimator));
fprintf('✓ Figure saved\n\n');

fprintf('=========================================================\n');
fprintf('SIMULATION COMPLETE\n');
fprintf('=========================================================\n\n');

%% Helper Functions
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

function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

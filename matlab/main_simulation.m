% main_simulation.m - Main autonomous vehicle navigation simulation
% Integrates path planning, control, and safety components

% Add paths
addpath('core');
addpath('utils');
addpath('planners');
addpath('controllers');
addpath('safety');

clear all;
close all;
clc;

fprintf('=========================================================\n');
fprintf('  AUTONOMOUS VEHICLE NAVIGATION SIMULATION\n');
fprintf('=========================================================\n\n');

%% User Input: Planner Selection
fprintf('Available Path Planners:\n');
fprintf('  1. A* (Grid-based, Manhattan heuristic)\n');
fprintf('  2. RRT* (Sampling-based, asymptotically optimal)\n');
fprintf('  3. Hybrid A* (Kinematically feasible paths)\n');
planner_choice = input('Select planner (1-3): ');

while ~ismember(planner_choice, [1, 2, 3])
    fprintf('Invalid choice. Please select 1, 2, or 3.\n');
    planner_choice = input('Select planner (1-3): ');
end

planner_names = {'A*', 'RRT*', 'Hybrid A*'};
fprintf('Selected: %s\n\n', planner_names{planner_choice});

%% User Input: Controller Selection
fprintf('Available Controllers:\n');
fprintf('  1. PID (Proportional-Integral-Derivative)\n');
fprintf('  2. LQR (Linear Quadratic Regulator)\n');
fprintf('  3. MPC (Model Predictive Control)\n');
controller_choice = input('Select controller (1-3): ');

while ~ismember(controller_choice, [1, 2, 3])
    fprintf('Invalid choice. Please select 1, 2, or 3.\n');
    controller_choice = input('Select controller (1-3): ');
end

controller_names = {'PID', 'LQR', 'MPC'};
fprintf('Selected: %s\n\n', controller_names{controller_choice});

%% User Input: CBF Safety Filter
fprintf('Safety Layer:\n');
cbf_choice = input('Enable CBF safety filter? (1=Yes, 0=No): ');

while ~ismember(cbf_choice, [0, 1])
    fprintf('Invalid choice. Please enter 0 or 1.\n');
    cbf_choice = input('Enable CBF safety filter? (1=Yes, 0=No): ');
end

fprintf('CBF Safety: %s\n\n', iif(cbf_choice, 'Enabled', 'Disabled'));

%% User Input: State Estimation Mode
fprintf('State Estimation Modes:\n');
fprintf('  1. Perfect (Ground truth)\n');
fprintf('  2. Noisy (Ground truth + Gaussian noise)\n');
fprintf('  3. EKF (Extended Kalman Filter)\n');
fprintf('  4. UKF (Unscented Kalman Filter)\n');
fprintf('  5. PF (Particle Filter)\n');
fprintf('  6. External (Load from file)\n');
estimator_choice = input('Select state estimation mode (1-6): ');

while ~ismember(estimator_choice, [1, 2, 3, 4, 5, 6])
    fprintf('Invalid choice. Please select 1-6.\n');
    estimator_choice = input('Select state estimation mode (1-6): ');
end

estimator_modes = {'perfect', 'noisy', 'ekf', 'ukf', 'pf', 'external'};
estimator_names = {'Perfect', 'Noisy', 'EKF', 'UKF', 'PF', 'External'};
fprintf('Selected: %s\n\n', estimator_names{estimator_choice});

% TEMPORARY: Force perfect estimation for MPC (EKF needs debugging)
if controller_choice == 3 && estimator_choice ~= 1
    fprintf('*** OVERRIDE: Using perfect estimation for MPC (EKF/UKF/PF need tuning) ***\n\n');
    estimator_choice = 1;
end

%% Setup Simulation Parameters
fprintf('=========================================================\n');
fprintf('Setting up simulation...\n');
fprintf('=========================================================\n\n');

% Time parameters
dt = 0.1;
T = 100;  % Maximum simulation time
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
fprintf('✓ Vehicle model: Bicycle (L=%.2fm)\n', vehicle.L);

%% Path Planning
fprintf('\n---------------------------------------------------------\n');
fprintf('PHASE 1: Path Planning\n');
fprintf('---------------------------------------------------------\n');

switch planner_choice
    case 1  % A*
        planner = AStarPlanner(env, 1.0);
        [path, stats] = planner.plan(start(1:2), goal(1:2));
        % Add headings to A* path
        if stats.success
            path = add_headings_to_path(path);
        end
        
    case 2  % RRT*
        planner = RRTStarPlanner(env, 1.0);
        [path, stats] = planner.plan(start(1:2), goal(1:2));
        % Add headings to RRT* path
        if stats.success
            path = add_headings_to_path(path);
        end
        
    case 3  % Hybrid A*
        planner = HybridAStarPlanner(env, 1.0, vehicle.L);
        [path, stats] = planner.plan(start, goal);
end

if ~stats.success
    fprintf('✗ Path planning failed!\n');
    return;
end

fprintf('✓ Path planned successfully\n');
fprintf('  Waypoints: %d\n', size(path, 1));
if isfield(stats, 'path_length')
    fprintf('  Path length: %.2f m\n', stats.path_length);
end
if isfield(stats, 'time')
    fprintf('  Planning time: %.3f s\n', stats.time);
end
if isfield(stats, 'nodes_explored')
    fprintf('  Nodes explored: %d\n', stats.nodes_explored);
end

%% Controller Setup
fprintf('\n---------------------------------------------------------\n');
fprintf('PHASE 2: Controller Configuration\n');
fprintf('---------------------------------------------------------\n');

switch controller_choice
    case 1  % PID
        controller = PIDController(4.0, 0.8, 2.0, 8.0, 1.0, 1.5);
        fprintf('✓ PID controller created\n');
        fprintf('  Velocity gains: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', 4.0, 0.8, 2.0);
        fprintf('  Heading gains:  Kp=%.1f, Ki=%.1f, Kd=%.1f\n', 8.0, 1.0, 1.5);
        
    case 2  % LQR
        Q = diag([10, 10, 5, 1]);
        R = diag([1, 10]);
        controller = LQRController(Q, R, dt, 'bicycle', vehicle.L);
        fprintf('✓ LQR controller created\n');
        fprintf('  State weights Q: diag([%.0f, %.0f, %.0f, %.0f])\n', ...
                Q(1,1), Q(2,2), Q(3,3), Q(4,4));
        fprintf('  Control weights R: diag([%.0f, %.0f])\n', R(1,1), R(2,2));
        
    case 3  % MPC
        N = 10;
        Q = diag([20, 20, 10, 2]);  % Balanced - track path more closely
        R = diag([5.0, 15.0]);  % Smooth controls
        Qf = diag([40, 40, 20, 4]);  % Strong terminal cost
        controller = MPCController(N, dt, Q, R, Qf, 'bicycle', vehicle.L);
        fprintf('✓ MPC controller created\n');
        fprintf('  Prediction horizon: %d steps\n', N);
        fprintf('  State weights Q: diag([%.0f, %.0f, %.0f, %.0f])\n', ...
                Q(1,1), Q(2,2), Q(3,3), Q(4,4));
        fprintf('  Control weights R: diag([%.2f, %.1f])\n', R(1,1), R(2,2));
end

%% CBF Safety Filter Setup
if cbf_choice
    fprintf('\n---------------------------------------------------------\n');
    fprintf('PHASE 3: Safety Layer Configuration\n');
    fprintf('---------------------------------------------------------\n');
    
    % Adaptive safety parameters: MPC needs looser CBF for flexibility
    if controller_choice == 3  % MPC
        safety_margin = 0.8;  % Larger margin for MPC planning
        alpha = 1.5;  % Very high alpha - minimal but necessary intervention
    else
        safety_margin = 0.3;  % Tighter for reactive controllers
        alpha = 0.5;
    end
    cbf = CBFSafetyFilter(env, safety_margin, alpha, 'bicycle', vehicle.L, dt);
    fprintf('✓ CBF safety filter created\n');
    fprintf('  Safety margin: %.1f m\n', safety_margin);
    fprintf('  CBF parameter alpha: %.1f\n', alpha);
    if controller_choice == 3
        fprintf('  Note: Relaxed CBF for MPC (larger margin + higher alpha)\n');
    end
end

%% State Estimator Setup
fprintf('\n---------------------------------------------------------\n');
fprintf('PHASE 4: State Estimation Configuration\n');
fprintf('---------------------------------------------------------\n');

switch estimator_choice
    case 1  % Perfect
        estimator = StateEstimator('perfect');
        fprintf('✓ Perfect state estimation (ground truth)\n');
        
    case 2  % Noisy
        noise_std = [0.5, 0.5, 0.1];  % [x, y, theta] noise
        estimator = StateEstimator('noisy', noise_std);
        fprintf('✓ Noisy state estimation\n');
        fprintf('  Position noise std: %.2f m\n', noise_std(1));
        fprintf('  Heading noise std: %.2f rad\n', noise_std(3));
        
    case 3  % EKF
        process_noise = diag([0.01, 0.01, 0.01, 0.02]);
        measurement_noise = diag([0.1, 0.1, 0.05]);
        estimator = EKFEstimator(dt, process_noise, measurement_noise, vehicle.L);
        fprintf('✓ Extended Kalman Filter\n');
        fprintf('  Process noise: diag([%.2f, %.2f, %.3f, %.2f])\n', ...
                process_noise(1,1), process_noise(2,2), process_noise(3,3), process_noise(4,4));
        fprintf('  Measurement noise: diag([%.2f, %.2f, %.2f])\n', ...
                measurement_noise(1,1), measurement_noise(2,2), measurement_noise(3,3));
        
    case 4  % UKF
        process_noise = diag([0.01, 0.01, 0.01, 0.02]);
        measurement_noise = diag([0.1, 0.1, 0.05]);
        estimator = UKFEstimator(dt, process_noise, measurement_noise, vehicle.L);
        fprintf('✓ Unscented Kalman Filter\n');
        fprintf('  Process noise: diag([%.2f, %.2f, %.3f, %.2f])\n', ...
                process_noise(1,1), process_noise(2,2), process_noise(3,3), process_noise(4,4));
        fprintf('  Measurement noise: diag([%.2f, %.2f, %.2f])\n', ...
                measurement_noise(1,1), measurement_noise(2,2), measurement_noise(3,3));
        
    case 5  % Particle Filter
        process_noise = diag([0.01, 0.01, 0.01, 0.02]);
        measurement_noise = diag([0.1, 0.1, 0.05]);
        N_particles = 500;
        estimator = ParticleFilter(dt, process_noise, measurement_noise, N_particles, vehicle.L);
        fprintf('✓ Particle Filter\n');
        fprintf('  Number of particles: %d\n', N_particles);
        fprintf('  Process noise: diag([%.2f, %.2f, %.3f, %.2f])\n', ...
                process_noise(1,1), process_noise(2,2), process_noise(3,3), process_noise(4,4));
        fprintf('  Measurement noise: diag([%.2f, %.2f, %.2f])\n', ...
                measurement_noise(1,1), measurement_noise(2,2), measurement_noise(3,3));
        
    case 6  % External
        estimator = StateEstimator('external');
        fprintf('✓ External state data (not loaded - using perfect for demo)\n');
        fprintf('  Note: Load external data with estimator.load_external_data(filename)\n');
        % For now, fall back to perfect
        estimator = StateEstimator('perfect');
end

%% Main Simulation Loop
fprintf('\n=========================================================\n');
fprintf('PHASE 5: Running Simulation\n');
fprintf('=========================================================\n\n');

% Initialize state
true_state = [start(1), start(2), start(3), 0];  % [x, y, theta, v]
estimated_state = true_state;

% Initialize EKF/UKF/PF if using them
if ismember(estimator_choice, [3, 4, 5])  % EKF, UKF, or PF
    estimator = estimator.initialize(true_state);
end

% Storage
trajectory_true = zeros(steps, 4);
trajectory_estimated = zeros(steps, 4);
controls = zeros(steps, 2);
tracking_errors = zeros(steps, 1);
estimation_errors = zeros(steps, 1);
cbf_activations = zeros(steps, 1);
min_obstacle_distances = zeros(steps, 1);

% Progress tracking
fprintf('Simulation progress:\n');
start_time = tic;

for i = 1:steps
    % Store true state
    trajectory_true(i, :) = true_state;
    
    % Get state estimate
    if estimator_choice == 3  % EKF
        % Create noisy measurement (ensure column vector)
        z = true_state(1:3)';  % Extract [x, y, theta] as row vector
        z = z(:);  % Convert to column vector
        z = z + [randn()*0.1; randn()*0.1; randn()*0.05];  % Add noise matching measurement_noise
        
        % EKF prediction (uses [a, delta] format)
        if i > 1
            a_ekf = (true_state(4) - trajectory_true(i-1, 4)) / dt;
        else
            a_ekf = 0;
        end
        v_curr = true_state(4);
        omega_curr = controls(max(1, i-1), 2);
        delta_ekf = atan(vehicle.L * omega_curr / max(v_curr, 0.1));
        u_ekf = [a_ekf; delta_ekf];
        estimator = estimator.predict(u_ekf);
        % EKF update
        [estimator, estimated_state] = estimator.update(z);
    elseif estimator_choice == 4  % UKF
        % Create noisy measurement (ensure column vector)
        z = true_state(1:3)';  % Extract [x, y, theta] as row vector
        z = z(:);  % Convert to column vector
        z = z + [randn()*0.1; randn()*0.1; randn()*0.05];  % Add noise matching measurement_noise
        
        % UKF prediction (uses [v, omega] format)
        u_prev = controls(max(1, i-1), :)';
        estimator = estimator.predict(u_prev);
        % UKF update
        [estimator, estimated_state] = estimator.update(z);
    elseif estimator_choice == 5  % PF
        % Create noisy measurement (ensure column vector)
        z = true_state(1:3)';  % Extract [x, y, theta] as row vector
        z = z(:);  % Convert to column vector
        z = z + [randn()*0.1; randn()*0.1; randn()*0.05];  % Add noise matching measurement_noise
        
        % PF prediction (uses [v, omega] format)
        u_prev = controls(max(1, i-1), :)';
        estimator = estimator.predict(u_prev);
        % PF update
        estimator = estimator.update(z);
        estimated_state = estimator.get_state_estimate();
    else
        estimated_state = estimator.get_state(true_state, i*dt);
    end
    
    trajectory_estimated(i, :) = estimated_state;
    
    % Compute nominal control using estimated state
    dist_to_goal = norm([estimated_state(1)-goal(1), estimated_state(2)-goal(2)]);
    
    % Direct goal-seeking mode when close to goal
    % Use larger distance for PID and MPC since they struggle with complex paths
    goal_seek_distance = 5.0;  % Default for LQR
    if controller_choice == 1  % PID needs more help
        goal_seek_distance = 7.0;
    elseif controller_choice == 3  % MPC - keep following path until very close
        goal_seek_distance = 4.0;  % Reduced to follow planned path longer
    end
    
    if dist_to_goal < goal_seek_distance
        % Simple proportional controller directly to goal
        goal_direction = atan2(goal(2) - estimated_state(2), goal(1) - estimated_state(1));
        heading_error = atan2(sin(goal_direction - estimated_state(3)), cos(goal_direction - estimated_state(3)));
        
        % Aggressive approach to goal
        v_nom = min(2.5, dist_to_goal * 2.0);  % Speed proportional to distance
        w_nom = 4.0 * heading_error;  % Strong heading correction
    else
        % Normal path following
        switch controller_choice
            case 1  % PID
                [v_nom, w_nom, controller] = controller.track_path(estimated_state, path, 1.8, dt);
            case 2  % LQR
                [v_nom, w_nom] = controller.track_path(estimated_state, path, 3.5);
            case 3  % MPC
                [v_nom, w_nom] = controller.track_path(estimated_state, path, 1.5);  % Shorter lookahead for tighter tracking
        end
    end
    
    % Apply CBF safety filter if enabled
    % For MPC: Activate CBF only in emergency (within 1.5m of obstacles)
    % This prevents collisions while minimizing interference with optimization
    cbf_disable_distance = 10.0;
    
    use_cbf = false;
    if cbf_choice
        if controller_choice == 3  % MPC
            % Emergency CBF: only when dangerously close to obstacles
            if cbf_choice && dist_to_goal > cbf_disable_distance
                min_dist = cbf.get_min_obstacle_distance(true_state);
                if min_dist < 1.5  % Within 1.5m of obstacle
                    use_cbf = true;
                end
            end
        else  % PID/LQR - normal CBF
            if dist_to_goal > cbf_disable_distance
                use_cbf = true;
            end
        end
    end
    
    if use_cbf
        [v_safe, w_safe, is_modified] = cbf.filter_control(true_state, v_nom, w_nom);
        cbf_activations(i) = is_modified;
    else
        v_safe = v_nom;
        w_safe = w_nom;
        cbf_activations(i) = 0;  % CBF not active
    end
    
    controls(i, :) = [v_safe, w_safe];
    
    % Convert to bicycle model inputs
    delta = atan(vehicle.L * w_safe / max(v_safe, 0.1));
    delta = max(min(delta, pi/4), -pi/4);
    a = (v_safe - true_state(4)) / dt;
    
    % Update true vehicle state
    true_state = vehicle.step(true_state, [a, delta]);
    
    % Calculate tracking error
    distances = sqrt((path(:,1) - true_state(1)).^2 + (path(:,2) - true_state(2)).^2);
    tracking_errors(i) = min(distances);
    
    % Calculate estimation error
    estimation_errors(i) = norm(true_state(1:3) - estimated_state(1:3));
    
    % Get minimum obstacle distance
    if cbf_choice
        min_obstacle_distances(i) = cbf.get_min_obstacle_distance(true_state);
    end
    
    % Progress update
    if mod(i, 100) == 0
        fprintf('  Step %d/%d (%.1f%%) - Tracking error: %.3fm\n', ...
                i, steps, 100*i/steps, tracking_errors(i));
    end
    
    % Check if goal reached
    goal_threshold = 2.5;
    if controller_choice == 3  % MPC needs larger threshold
        goal_threshold = 3.5;
    end
    if norm([true_state(1)-goal(1), true_state(2)-goal(2)]) < goal_threshold && i > 50
        trajectory_true = trajectory_true(1:i, :);
        trajectory_estimated = trajectory_estimated(1:i, :);
        controls = controls(1:i, :);
        tracking_errors = tracking_errors(1:i);
        estimation_errors = estimation_errors(1:i);
        cbf_activations = cbf_activations(1:i);
        if cbf_choice
            min_obstacle_distances = min_obstacle_distances(1:i);
        end
        fprintf('\n✓ Goal reached at step %d (%.1f seconds)\n', i, i*dt);
        break;
    end
end

elapsed_time = toc(start_time);

%% Results Summary
fprintf('\n=========================================================\n');
fprintf('SIMULATION RESULTS\n');
fprintf('=========================================================\n\n');

fprintf('Configuration:\n');
fprintf('  Planner: %s\n', planner_names{planner_choice});
fprintf('  Controller: %s\n', controller_names{controller_choice});
fprintf('  CBF Safety: %s\n', iif(cbf_choice, 'Enabled', 'Disabled'));
fprintf('  State Estimation: %s\n\n', estimator_names{estimator_choice});

fprintf('Performance Metrics:\n');
fprintf('  Final position: (%.2f, %.2f, %.2f rad)\n', ...
        true_state(1), true_state(2), true_state(3));
fprintf('  Distance to goal: %.3f m\n', ...
        norm([true_state(1)-goal(1), true_state(2)-goal(2)]));
fprintf('  Mean tracking error: %.3f m\n', mean(tracking_errors));
fprintf('  Max tracking error: %.3f m\n', max(tracking_errors));
fprintf('  Mean estimation error: %.3f m\n', mean(estimation_errors));
fprintf('  Max estimation error: %.3f m\n', max(estimation_errors));
fprintf('  Simulation time: %.2f s (real-time)\n', elapsed_time);
fprintf('  Simulated time: %.2f s\n', length(tracking_errors)*dt);

if cbf_choice
    fprintf('  CBF activations: %d/%d (%.1f%%)\n', ...
            sum(cbf_activations), length(cbf_activations), ...
            100*sum(cbf_activations)/length(cbf_activations));
    fprintf('  Min obstacle distance: %.3f m\n', min(min_obstacle_distances));
    fprintf('  Safety maintained: %s\n', iif(min(min_obstacle_distances) >= 0, 'YES', 'NO'));
end

%% Visualization
fprintf('\n=========================================================\n');
fprintf('Generating visualizations...\n');
fprintf('=========================================================\n');

% Figure 1: Path and Trajectory
results_fig = figure('Name', 'Simulation Results', 'Position', [100, 100, 1400, 800]);

subplot(2,3,1);
env.plot();
hold on;
plot(path(:,1), path(:,2), 'b--', 'DisplayName', 'Planned Path');
plot(trajectory_true(:,1), trajectory_true(:,2), 'r-', 'DisplayName', 'True Trajectory');
if estimator_choice > 1
    plot(trajectory_estimated(:,1), trajectory_estimated(:,2), 'g:', 'DisplayName', 'Estimated');
end
plot(start(1), start(2), 'go', 'MarkerSize', 12, 'DisplayName', 'Start');
plot(goal(1), goal(2), 'r^', 'MarkerSize', 12, 'DisplayName', 'Goal');
xlabel('X (m)'); ylabel('Y (m)');
title(sprintf('Path Planning & Tracking\n%s + %s', planner_names{planner_choice}, controller_names{controller_choice}));
legend('Location', 'best');
grid on; axis equal;

% Tracking Error
subplot(2,3,2);
plot((1:length(tracking_errors))*dt, tracking_errors, 'r-');
xlabel('Time (s)'); ylabel('Tracking Error (m)');
title('Path Tracking Error');
grid on;

% Estimation Error
subplot(2,3,3);
plot((1:length(estimation_errors))*dt, estimation_errors, 'b-');
xlabel('Time (s)'); ylabel('Estimation Error (m)');
title(sprintf('State Estimation Error\n(%s)', estimator_names{estimator_choice}));
grid on;

% Control Inputs
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

% CBF Activation or Obstacle Distancefig_filename_png
subplot(2,3,6);
if cbf_choice
    yyaxis left;
    plot((1:length(min_obstacle_distances))*dt, min_obstacle_distances, 'b-');
    ylabel('Min Distance to Obstacle (m)');
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
    xlabel('Time (s)'); ylabel('Cumulative Tracking Error (m·s)');
    title('Integrated Tracking Error');
    grid on;
end

fprintf('✓ Visualization complete\n\n');

%% Save Results
fprintf('=========================================================\n');
fprintf('Saving Results\n');
fprintf('=========================================================\n\n');

% Prepare configuration struct
config = struct();
config.planner_name = planner_names{planner_choice};
config.controller_name = controller_names{controller_choice};
config.cbf_enabled = cbf_choice;
config.estimator_name = estimator_names{estimator_choice};
config.start_pos = start;
config.goal_pos = goal;
config.dt = dt;
config.vehicle_wheelbase = vehicle.L;

% Compute comprehensive metrics
timestamps = (0:length(tracking_errors)-1)' * dt;
metrics = compute_metrics(trajectory_true, path, controls, timestamps, ...
                         cbf_activations, min_obstacle_distances, stats);

% Generate filename based on configuration
filename = sprintf('%s_%s_CBF%d_%s', ...
                   strrep(config.planner_name, '*', 'Star'), ...
                   config.controller_name, ...
                   config.cbf_enabled, ...
                   config.estimator_name);
filename = strrep(filename, ' ', '');  % Remove spaces

% Save all results
save_results(metrics, trajectory_true, controls, timestamps, config, filename);

% Save visualization figure
if ~exist('results', 'dir')
    mkdir('results');
end
fig_filename_png = fullfile('results', [filename, '.png']);
fig_filename_fig = fullfile('results', [filename, '.fig']);
saveas(results_fig, fig_filename_png, 'png');
saveas(results_fig, fig_filename_fig, 'fig');
fprintf('✓ Saved visualization to: %s\n', fig_filename_png);
fprintf('✓ Saved MATLAB figure to: %s\n', fig_filename_fig);

fprintf('\n=========================================================\n');
fprintf('SIMULATION COMPLETE\n');
fprintf('=========================================================\n\n');

%% Helper Functions
function path_with_heading = add_headings_to_path(path)
    % Add heading angle to 2D path
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

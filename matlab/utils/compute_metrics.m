function metrics = compute_metrics(trajectory, path, controls, timestamps, cbf_activations, min_distances, planner_stats)
    % compute_metrics - Calculate comprehensive simulation performance metrics
    %
    % Inputs:
    %   trajectory: Nx4 matrix [x, y, theta, v] of actual trajectory
    %   path: Mx3 matrix [x, y, theta] of planned path
    %   controls: Nx2 matrix [v, omega] of control inputs
    %   timestamps: Nx1 vector of simulation times
    %   cbf_activations: Nx1 binary vector indicating CBF activation
    %   min_distances: Nx1 vector of minimum obstacle distances
    %   planner_stats: struct with planning statistics
    %
    % Output:
    %   metrics: struct containing all computed metrics
    
    metrics = struct();
    
    %% Path Planning Metrics
    if isfield(planner_stats, 'path_length')
        metrics.path_length = planner_stats.path_length;
    else
        % Calculate path length
        path_segments = diff(path(:,1:2), 1, 1);
        metrics.path_length = sum(sqrt(sum(path_segments.^2, 2)));
    end
    
    if isfield(planner_stats, 'time')
        metrics.planning_time = planner_stats.time;
    else
        metrics.planning_time = 0;
    end
    
    if isfield(planner_stats, 'nodes_explored')
        metrics.nodes_explored = planner_stats.nodes_explored;
    end
    
    if isfield(planner_stats, 'success')
        metrics.planning_success = planner_stats.success;
    else
        metrics.planning_success = true;
    end
    
    %% Trajectory Metrics
    % Actual trajectory length
    traj_segments = diff(trajectory(:,1:2), 1, 1);
    metrics.trajectory_length = sum(sqrt(sum(traj_segments.^2, 2)));
    
    % Tracking errors
    tracking_errors = zeros(size(trajectory, 1), 1);
    for i = 1:size(trajectory, 1)
        distances = sqrt((path(:,1) - trajectory(i,1)).^2 + (path(:,2) - trajectory(i,2)).^2);
        tracking_errors(i) = min(distances);
    end
    
    metrics.mean_tracking_error = mean(tracking_errors);
    metrics.max_tracking_error = max(tracking_errors);
    metrics.rms_tracking_error = sqrt(mean(tracking_errors.^2));
    metrics.final_tracking_error = tracking_errors(end);
    
    %% Path Smoothness Metrics
    % Curvature (rate of heading change)
    heading = trajectory(:,3);
    curvature = diff(heading) ./ diff(timestamps);
    metrics.mean_curvature = mean(abs(curvature));
    metrics.max_curvature = max(abs(curvature));
    
    % Velocity smoothness (jerk)
    velocity = trajectory(:,4);
    acceleration = diff(velocity) ./ diff(timestamps);
    jerk = diff(acceleration) ./ diff(timestamps(1:end-1));
    metrics.mean_jerk = mean(abs(jerk));
    metrics.max_jerk = max(abs(jerk));
    
    %% Control Performance Metrics
    % Control smoothness
    control_velocity = controls(:,1);
    control_omega = controls(:,2);
    
    vel_changes = diff(control_velocity) ./ diff(timestamps);
    omega_changes = diff(control_omega) ./ diff(timestamps);
    
    metrics.mean_velocity_change = mean(abs(vel_changes));
    metrics.max_velocity_change = max(abs(vel_changes));
    metrics.mean_omega_change = mean(abs(omega_changes));
    metrics.max_omega_change = max(abs(omega_changes));
    
    % Control effort
    metrics.total_velocity_effort = sum(abs(control_velocity) .* diff([0; timestamps]));
    metrics.total_omega_effort = sum(abs(control_omega) .* diff([0; timestamps]));
    
    %% Safety Metrics
    if ~isempty(cbf_activations)
        metrics.cbf_activation_count = sum(cbf_activations);
        metrics.cbf_activation_rate = mean(cbf_activations) * 100; % percentage
        metrics.cbf_duration = sum(cbf_activations) * mean(diff(timestamps));
    end
    
    if ~isempty(min_distances)
        metrics.min_obstacle_distance = min(min_distances);
        metrics.mean_obstacle_distance = mean(min_distances);
        metrics.safety_violations = sum(min_distances < 0);
        metrics.safety_maintained = (metrics.safety_violations == 0);
    end
    
    %% Timing Metrics
    metrics.simulation_duration = timestamps(end) - timestamps(1);
    metrics.num_steps = length(timestamps);
    metrics.mean_step_time = mean(diff(timestamps));
    
    %% Success Metrics
    % Consider success if final tracking error is small
    metrics.execution_success = (metrics.final_tracking_error < 3.0);
    metrics.overall_success = metrics.planning_success && metrics.execution_success;
    
    %% Efficiency Metrics
    % Path efficiency (ratio of straight-line distance to actual path)
    start_pos = trajectory(1, 1:2);
    end_pos = trajectory(end, 1:2);
    straight_line_dist = norm(end_pos - start_pos);
    metrics.path_efficiency = straight_line_dist / metrics.trajectory_length;
    
    % Time efficiency (average velocity)
    metrics.avg_velocity = metrics.trajectory_length / metrics.simulation_duration;
    
end

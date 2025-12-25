function fig = plot_simulation_results(env, path, trajectory_true, trajectory_est, controls, ...
                                       tracking_errors, estimation_errors, cbf_activations, ...
                                       min_distances, timestamps, config)
    % plot_simulation_results - Generate comprehensive visualization of simulation results
    %
    % Inputs:
    %   env: Environment object
    %   path: Planned path (Mx3: x, y, theta)
    %   trajectory_true: True trajectory (Nx4: x, y, theta, v)
    %   trajectory_est: Estimated trajectory (Nx4)
    %   controls: Control inputs (Nx2: v, omega)
    %   tracking_errors: Tracking errors over time (Nx1)
    %   estimation_errors: Estimation errors over time (Nx1)
    %   cbf_activations: CBF activation flags (Nx1)
    %   min_distances: Minimum obstacle distances (Nx1)
    %   timestamps: Time vector (Nx1)
    %   config: Configuration struct with fields:
    %           - planner_name
    %           - controller_name
    %           - cbf_enabled
    %           - estimator_name
    %           - start_pos
    %           - goal_pos
    %
    % Output:
    %   fig: Figure handle
    
    fig = figure('Name', 'Simulation Results', 'Position', [100, 100, 1400, 800]);
    
    %% Subplot 1: Environment and Trajectories
    subplot(2,3,1);
    env.plot();
    hold on;
    
    % Plot planned path
    plot(path(:,1), path(:,2), 'b--', 'DisplayName', 'Planned Path');
    
    % Plot true trajectory
    plot(trajectory_true(:,1), trajectory_true(:,2), 'r-', ...
         'DisplayName', 'True Trajectory');
    
    % Plot estimated trajectory if different from true
    if ~isequal(trajectory_est, trajectory_true)
        plot(trajectory_est(:,1), trajectory_est(:,2), 'g:', ...
             'DisplayName', 'Estimated Traj.');
    end
    
    % Plot start and goal
    plot(config.start_pos(1), config.start_pos(2), 'go', 'MarkerSize', 12, ...
         'MarkerFaceColor', 'g', 'DisplayName', 'Start');
    plot(config.goal_pos(1), config.goal_pos(2), 'r^', 'MarkerSize', 12, ...
         'MarkerFaceColor', 'r', 'DisplayName', 'Goal');
    
    xlabel('X (m)');
    ylabel('Y (m)');
    title(sprintf('Path Planning & Tracking\n%s + %s', ...
                  config.planner_name, config.controller_name));
    legend('Location', 'best');
    grid on;
    axis equal;
    
    %% Subplot 2: Tracking Error
    subplot(2,3,2);
    plot(timestamps, tracking_errors, 'r-');
    xlabel('Time (s)');
    ylabel('Tracking Error (m)');
    title('Path Tracking Error');
    grid on;
    
    % Add statistics
    mean_err = mean(tracking_errors);
    max_err = max(tracking_errors);
    text(0.02, 0.98, sprintf('Mean: %.3f m\nMax: %.3f m', mean_err, max_err), ...
         'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'BackgroundColor', 'white');
    
    %% Subplot 3: Estimation Error
    subplot(2,3,3);
    plot(timestamps, estimation_errors, 'b-');
    xlabel('Time (s)');
    ylabel('Estimation Error (m)');
    title(sprintf('State Estimation Error\n(%s)', config.estimator_name));
    grid on;
    
    % Add statistics
    mean_est_err = mean(estimation_errors);
    max_est_err = max(estimation_errors);
    text(0.02, 0.98, sprintf('Mean: %.3f m\nMax: %.3f m', mean_est_err, max_est_err), ...
         'Units', 'normalized', 'VerticalAlignment', 'top', ...
         'BackgroundColor', 'white');
    
    %% Subplot 4: Linear Velocity
    subplot(2,3,4);
    plot(timestamps, controls(:,1), 'b-');
    xlabel('Time (s)');
    ylabel('Linear Velocity (m/s)');
    title('Control Input - Linear Velocity');
    grid on;
    ylim([min(0, min(controls(:,1))-0.5), max(controls(:,1))+0.5]);
    
    %% Subplot 5: Angular Velocity
    subplot(2,3,5);
    plot(timestamps, controls(:,2), 'r-');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    title('Control Input - Angular Velocity');
    grid on;
    
    %% Subplot 6: Safety Monitoring
    subplot(2,3,6);
    if config.cbf_enabled && ~isempty(cbf_activations) && ~isempty(min_distances)
        yyaxis left;
        plot(timestamps, min_distances, 'b-');
        ylabel('Min Distance to Obstacle (m)');
        hold on;
        plot([timestamps(1), timestamps(end)], [0, 0], 'r--');
        
        yyaxis right;
        plot(timestamps, cbf_activations, 'r-');
        ylabel('CBF Active');
        ylim([-0.1, 1.1]);
        
        xlabel('Time (s)');
        title('CBF Safety Monitor');
        grid on;
        
        % Add statistics
        activation_rate = mean(cbf_activations) * 100;
        min_dist = min(min_distances);
        text(0.02, 0.98, sprintf('CBF Active: %.1f%%\nMin Dist: %.3f m', ...
                                 activation_rate, min_dist), ...
             'Units', 'normalized', 'VerticalAlignment', 'top', ...
             'BackgroundColor', 'white');
    else
        % Plot cumulative tracking error
        cumulative_error = cumsum(tracking_errors) .* diff([0; timestamps]);
        plot(timestamps, cumulative_error, 'g-');
        xlabel('Time (s)');
        ylabel('Cumulative Tracking Error (m·s)');
        title('Integrated Tracking Error');
        grid on;
    end
    
    % Overall title
    sgtitle(sprintf('Simulation Results: %s | %s | CBF: %s | Estimator: %s', ...
                    config.planner_name, config.controller_name, ...
                    iif(config.cbf_enabled, 'ON', 'OFF'), config.estimator_name), ...
            'FontSize', 14, 'FontWeight', 'bold');
end

function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

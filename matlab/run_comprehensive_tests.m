%% COMPREHENSIVE TESTING SUITE
% Tests multiple combinations of planners, controllers, CBF, and estimators

clear all; close all; clc;

% Add paths
addpath('planners');
addpath('controllers');
addpath('safety');
addpath('utils');
addpath('models');

fprintf('=========================================================\n');
fprintf('  COMPREHENSIVE AUTONOMOUS NAVIGATION TESTING\n');
fprintf('=========================================================\n\n');

% Test configuration matrix
test_configs = [
    % planner, controller, cbf, estimator, description
    3, 2, 1, 1;  % Hybrid A* + LQR + CBF + Perfect (baseline)
    3, 3, 1, 1;  % Hybrid A* + MPC + CBF + Perfect
    3, 2, 1, 3;  % Hybrid A* + LQR + CBF + EKF
    2, 2, 1, 1;  % RRT* + LQR + CBF + Perfect
    1, 2, 1, 1;  % A* + LQR + CBF + Perfect
    3, 2, 0, 1;  % Hybrid A* + LQR + No CBF + Perfect
];

test_names = {
    'Hybrid A* + LQR + CBF + Perfect (Baseline)',
    'Hybrid A* + MPC + CBF + Perfect',
    'Hybrid A* + LQR + CBF + EKF',
    'RRT* + LQR + CBF + Perfect',
    'A* + LQR + CBF + Perfect',
    'Hybrid A* + LQR + No CBF + Perfect'
};

num_tests = size(test_configs, 1);

% Storage for results
results = struct();
results.test_names = test_names;
results.configs = test_configs;
results.goal_distances = zeros(num_tests, 1);
results.mean_tracking_errors = zeros(num_tests, 1);
results.max_tracking_errors = zeros(num_tests, 1);
results.mean_estimation_errors = zeros(num_tests, 1);
results.cbf_activation_rates = zeros(num_tests, 1);
results.min_obstacle_distances = zeros(num_tests, 1);
results.simulation_times = zeros(num_tests, 1);
results.success = false(num_tests, 1);

% Run tests
for test_idx = 1:num_tests
    fprintf('\n\n');
    fprintf('*********************************************************\n');
    fprintf('  TEST %d/%d: %s\n', test_idx, num_tests, test_names{test_idx});
    fprintf('*********************************************************\n\n');
    
    % Set configuration
    planner = test_configs(test_idx, 1);
    controller = test_configs(test_idx, 2);
    cbf_enabled = test_configs(test_idx, 3);
    estimator = test_configs(test_idx, 4);
    
    try
        % Run simulation
        run('run_simulation.m');
        
        % Store results
        results.goal_distances(test_idx) = norm([true_state(1)-goal(1), true_state(2)-goal(2)]);
        results.mean_tracking_errors(test_idx) = mean(tracking_errors);
        results.max_tracking_errors(test_idx) = max(tracking_errors);
        results.mean_estimation_errors(test_idx) = mean(estimation_errors);
        
        if cbf_enabled
            results.cbf_activation_rates(test_idx) = 100*sum(cbf_activations)/length(cbf_activations);
            results.min_obstacle_distances(test_idx) = min(min_obstacle_distances);
        else
            results.cbf_activation_rates(test_idx) = 0;
            results.min_obstacle_distances(test_idx) = NaN;
        end
        
        results.simulation_times(test_idx) = elapsed_time;
        results.success(test_idx) = results.goal_distances(test_idx) < 2.0;
        
        fprintf('\n✓ Test %d PASSED\n', test_idx);
        
    catch ME
        fprintf('\n✗ Test %d FAILED: %s\n', test_idx, ME.message);
        results.success(test_idx) = false;
    end
    
    % Close figures to save memory
    close all;
end

%% Generate Summary Report
fprintf('\n\n');
fprintf('=========================================================\n');
fprintf('  COMPREHENSIVE TEST RESULTS SUMMARY\n');
fprintf('=========================================================\n\n');

fprintf('Overall Success Rate: %d/%d tests passed (%.1f%%)\n\n', ...
    sum(results.success), num_tests, 100*sum(results.success)/num_tests);

fprintf('%-50s | Goal Dist | Tracking | CBF Act | Min Obs Dist | Time | Status\n', 'Test Configuration');
fprintf('%s\n', repmat('-', 1, 120));

for i = 1:num_tests
    if isnan(results.min_obstacle_distances(i))
        obs_dist_str = '    N/A     ';
    else
        obs_dist_str = sprintf('   %.3f m   ', results.min_obstacle_distances(i));
    end
    
    fprintf('%-50s |   %.3f m  |  %.3f m  |  %5.1f%% | %s | %.2f s | %s\n', ...
        test_names{i}, ...
        results.goal_distances(i), ...
        results.mean_tracking_errors(i), ...
        results.cbf_activation_rates(i), ...
        obs_dist_str, ...
        results.simulation_times(i), ...
        iif(results.success(i), '✓ PASS', '✗ FAIL'));
end

fprintf('\n');

%% Performance Comparison Visualization
figure('Position', [100, 100, 1400, 900]);

% Goal reaching performance
subplot(2, 3, 1);
bar(results.goal_distances);
ylabel('Distance to Goal (m)');
xlabel('Test Number');
title('Goal Reaching Performance');
grid on;
set(gca, 'XTick', 1:num_tests);

% Tracking error
subplot(2, 3, 2);
bar(results.mean_tracking_errors);
ylabel('Mean Tracking Error (m)');
xlabel('Test Number');
title('Path Tracking Accuracy');
grid on;
set(gca, 'XTick', 1:num_tests);

% CBF activation
subplot(2, 3, 3);
cbf_data = results.cbf_activation_rates;
cbf_data(cbf_data == 0) = NaN;  % Don't show tests without CBF
bar(cbf_data);
ylabel('CBF Activation Rate (%)');
xlabel('Test Number');
title('Safety Layer Activity');
grid on;
set(gca, 'XTick', 1:num_tests);

% Obstacle avoidance
subplot(2, 3, 4);
obs_data = results.min_obstacle_distances;
bar(obs_data);
ylabel('Min Obstacle Distance (m)');
xlabel('Test Number');
title('Collision Avoidance');
grid on;
set(gca, 'XTick', 1:num_tests);
hold on;
plot([0, num_tests+1], [0.5, 0.5], 'r--', 'DisplayName', 'Safety Margin');

% Computation time
subplot(2, 3, 5);
bar(results.simulation_times);
ylabel('Simulation Time (s)');
xlabel('Test Number');
title('Computational Performance');
grid on;
set(gca, 'XTick', 1:num_tests);

% Success summary
subplot(2, 3, 6);
success_data = double(results.success);
bar(success_data);
ylabel('Success (1=Pass, 0=Fail)');
xlabel('Test Number');
title('Test Success Status');
grid on;
set(gca, 'XTick', 1:num_tests, 'YTick', [0, 1]);
ylim([-0.2, 1.2]);

sgtitle('Comprehensive Test Results Comparison', 'FontSize', 14, 'FontWeight', 'bold');

% Save results
saveas(gcf, 'comprehensive_test_results.png');
save('test_results.mat', 'results');

fprintf('✓ Results saved to test_results.mat and comprehensive_test_results.png\n\n');

fprintf('=========================================================\n');
fprintf('  TESTING COMPLETE\n');
fprintf('=========================================================\n\n');

%% Helper function
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

%% Results Viewer - Load and Display Previous Test Results

clear all; close all; clc;

fprintf('=========================================================\n');
fprintf('  SIMULATION RESULTS VIEWER\n');
fprintf('=========================================================\n\n');

% Check if results file exists
if ~exist('test_results.mat', 'file')
    fprintf('No results file found.\n');
    fprintf('Please run comprehensive tests first:\n');
    fprintf('  >> run_comprehensive_tests\n\n');
    return;
end

% Load results
load('test_results.mat', 'results');

fprintf('Loaded test results with %d configurations\n\n', length(results.test_names));

%% Display Summary Table
fprintf('=========================================================\n');
fprintf('  TEST RESULTS SUMMARY\n');
fprintf('=========================================================\n\n');

fprintf('Success Rate: %d/%d tests (%.1f%%)\n\n', ...
    sum(results.success), length(results.success), ...
    100*sum(results.success)/length(results.success));

fprintf('%-50s | Goal  | Track | CBF   | Obs Dist | Time  | Status\n', 'Configuration');
fprintf('%s\n', repmat('-', 1, 120));

for i = 1:length(results.test_names)
    if isnan(results.min_obstacle_distances(i))
        obs_str = '   N/A   ';
    else
        obs_str = sprintf(' %.3f m ', results.min_obstacle_distances(i));
    end
    
    fprintf('%-50s | %.3fm | %.3fm | %4.1f%% | %s | %.2fs | %s\n', ...
        results.test_names{i}, ...
        results.goal_distances(i), ...
        results.mean_tracking_errors(i), ...
        results.cbf_activation_rates(i), ...
        obs_str, ...
        results.simulation_times(i), ...
        iif(results.success(i), '✓ PASS', '✗ FAIL'));
end

fprintf('\n');

%% Find Best Configuration
[~, best_idx] = min(results.goal_distances);
fprintf('BEST CONFIGURATION: %s\n', results.test_names{best_idx});
fprintf('  Goal Distance: %.3f m\n', results.goal_distances(best_idx));
fprintf('  Tracking Error: %.3f m\n', results.mean_tracking_errors(best_idx));
fprintf('  CBF Activation: %.1f%%\n', results.cbf_activation_rates(best_idx));
if ~isnan(results.min_obstacle_distances(best_idx))
    fprintf('  Min Obstacle Distance: %.3f m\n', results.min_obstacle_distances(best_idx));
end
fprintf('  Computation Time: %.2f s\n\n', results.simulation_times(best_idx));

%% Performance Rankings
fprintf('=========================================================\n');
fprintf('  PERFORMANCE RANKINGS\n');
fprintf('=========================================================\n\n');

% Goal reaching
[sorted_goals, goal_idx] = sort(results.goal_distances);
fprintf('Best Goal Reaching:\n');
for i = 1:min(3, length(goal_idx))
    idx = goal_idx(i);
    fprintf('  %d. %s (%.3f m)\n', i, results.test_names{idx}, sorted_goals(i));
end
fprintf('\n');

% Tracking accuracy
[sorted_tracking, track_idx] = sort(results.mean_tracking_errors);
fprintf('Best Path Tracking:\n');
for i = 1:min(3, length(track_idx))
    idx = track_idx(i);
    fprintf('  %d. %s (%.3f m)\n', i, results.test_names{idx}, sorted_tracking(i));
end
fprintf('\n');

% Computation speed
[sorted_time, time_idx] = sort(results.simulation_times);
fprintf('Fastest Computation:\n');
for i = 1:min(3, length(time_idx))
    idx = time_idx(i);
    fprintf('  %d. %s (%.2f s)\n', i, results.test_names{idx}, sorted_time(i));
end
fprintf('\n');

%% Regenerate Comparison Plots
fprintf('Generating comparison plots...\n');

figure('Position', [100, 100, 1400, 900]);

% Goal reaching performance
subplot(2, 3, 1);
bar(results.goal_distances);
ylabel('Distance to Goal (m)');
xlabel('Test Configuration');
title('Goal Reaching Performance');
grid on;
set(gca, 'XTick', 1:length(results.test_names));
set(gca, 'XTickLabel', []);  % Too many labels

% Tracking error
subplot(2, 3, 2);
bar(results.mean_tracking_errors);
ylabel('Mean Tracking Error (m)');
xlabel('Test Configuration');
title('Path Tracking Accuracy');
grid on;
set(gca, 'XTick', 1:length(results.test_names));
set(gca, 'XTickLabel', []);

% CBF activation
subplot(2, 3, 3);
cbf_data = results.cbf_activation_rates;
cbf_data(cbf_data == 0) = NaN;
bar(cbf_data);
ylabel('CBF Activation Rate (%)');
xlabel('Test Configuration');
title('Safety Layer Activity');
grid on;
set(gca, 'XTick', 1:length(results.test_names));
set(gca, 'XTickLabel', []);

% Obstacle avoidance
subplot(2, 3, 4);
bar(results.min_obstacle_distances);
ylabel('Min Obstacle Distance (m)');
xlabel('Test Configuration');
title('Collision Avoidance');
grid on;
hold on;
plot([0, length(results.test_names)+1], [0.5, 0.5], 'r--', 'DisplayName', 'Safety Margin');
set(gca, 'XTick', 1:length(results.test_names));
set(gca, 'XTickLabel', []);

% Computation time
subplot(2, 3, 5);
bar(results.simulation_times);
ylabel('Simulation Time (s)');
xlabel('Test Configuration');
title('Computational Performance');
grid on;
set(gca, 'XTick', 1:length(results.test_names));
set(gca, 'XTickLabel', []);

% Success rate
subplot(2, 3, 6);
success_data = double(results.success);
bar(success_data);
ylabel('Success (1=Pass, 0=Fail)');
xlabel('Test Configuration');
title('Test Success Status');
grid on;
set(gca, 'XTick', 1:length(results.test_names), 'YTick', [0, 1]);
ylim([-0.2, 1.2]);
set(gca, 'XTickLabel', []);

sgtitle('Comprehensive Test Results Comparison', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('✓ Plots generated\n\n');

%% Configuration Details
fprintf('=========================================================\n');
fprintf('  DETAILED CONFIGURATION MATRIX\n');
fprintf('=========================================================\n\n');

fprintf('Test# | Planner    | Controller | CBF | Estimator | Status\n');
fprintf('------|------------|------------|-----|-----------|--------\n');

planner_names = {'A*', 'RRT*', 'Hybrid A*'};
controller_names = {'PID', 'LQR', 'MPC'};
cbf_names = {'No', 'Yes'};
estimator_names = {'Perfect', 'Noisy', 'EKF', 'External'};

for i = 1:size(results.configs, 1)
    fprintf('  %d   | %-10s | %-10s | %-3s | %-9s | %s\n', ...
        i, ...
        planner_names{results.configs(i, 1)}, ...
        controller_names{results.configs(i, 2)}, ...
        cbf_names{results.configs(i, 3) + 1}, ...
        estimator_names{results.configs(i, 4)}, ...
        iif(results.success(i), '✓ PASS', '✗ FAIL'));
end

fprintf('\n');

%% Recommendations
fprintf('=========================================================\n');
fprintf('  RECOMMENDATIONS\n');
fprintf('=========================================================\n\n');

successful_idx = find(results.success);

if isempty(successful_idx)
    fprintf('No successful configurations found.\n');
    fprintf('Please check test parameters and re-run tests.\n\n');
else
    % Find best successful config
    [~, best_success_idx] = min(results.goal_distances(successful_idx));
    best_config = successful_idx(best_success_idx);
    
    fprintf('RECOMMENDED CONFIGURATION:\n');
    fprintf('  Configuration: %s\n', results.test_names{best_config});
    fprintf('  Planner: %s\n', planner_names{results.configs(best_config, 1)});
    fprintf('  Controller: %s\n', controller_names{results.configs(best_config, 2)});
    fprintf('  CBF Safety: %s\n', cbf_names{results.configs(best_config, 3) + 1});
    fprintf('  State Estimator: %s\n', estimator_names{results.configs(best_config, 4)});
    fprintf('\n');
    fprintf('  Performance:\n');
    fprintf('    - Goal Distance: %.3f m\n', results.goal_distances(best_config));
    fprintf('    - Tracking Error: %.3f m\n', results.mean_tracking_errors(best_config));
    fprintf('    - Computation Time: %.2f s\n', results.simulation_times(best_config));
    if results.configs(best_config, 3) == 1
        fprintf('    - Safety Margin: %.3f m\n', results.min_obstacle_distances(best_config));
        fprintf('    - CBF Activation: %.1f%%\n', results.cbf_activation_rates(best_config));
    end
    fprintf('\n');
    
    fprintf('To run this configuration:\n');
    fprintf('  planner=%d; controller=%d; cbf_enabled=%d; estimator=%d;\n', ...
        results.configs(best_config, 1), ...
        results.configs(best_config, 2), ...
        results.configs(best_config, 3), ...
        results.configs(best_config, 4));
    fprintf('  run(''run_simulation.m'');\n\n');
end

fprintf('=========================================================\n\n');

%% Helper function
function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

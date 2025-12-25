% test_estimators.m - Comprehensive test for EKF, UKF, and Particle Filter
% Compares all three estimators on a simulated trajectory

clear all;
close all;
clc;

addpath('core');
addpath('utils');

fprintf('=========================================================\n');
fprintf('  STATE ESTIMATOR COMPARISON TEST\n');
fprintf('=========================================================\n\n');

%% Simulation Parameters
dt = 0.1;  % Time step
T = 30;    % Simulation time
steps = round(T / dt);

% Vehicle parameters
L = 2.5;  % Wheelbase

% True trajectory: figure-8 pattern
t = (0:dt:T-dt)';
true_x = 5 * sin(0.2 * t);
true_y = 2.5 * sin(0.4 * t);
true_theta = atan2(diff([true_y; true_y(end)]), diff([true_x; true_x(end)]));
true_v = sqrt(diff([true_x; true_x(end)]).^2 + diff([true_y; true_y(end)]).^2) / dt;

% Noise parameters
process_noise_cov = diag([0.1, 0.1, 0.05, 0.1]);
measurement_noise_cov = diag([0.5, 0.5, 0.1]);

%% Initialize Estimators
fprintf('Initializing estimators...\n');

% EKF
ekf = EKFEstimator(dt, process_noise_cov, measurement_noise_cov);
ekf = ekf.initialize([true_x(1); true_y(1); true_theta(1); true_v(1)]);
fprintf('✓ EKF initialized\n');

% UKF  
ukf = UKFEstimator(dt, process_noise_cov, measurement_noise_cov, L);
ukf = ukf.initialize([true_x(1); true_y(1); true_theta(1); true_v(1)]);
fprintf('✓ UKF initialized\n');

% Particle Filter (100 particles for faster testing)
pf = ParticleFilter(dt, process_noise_cov, measurement_noise_cov, 100, L);
pf = pf.initialize([true_x(1); true_y(1); true_theta(1); true_v(1)], 0.3);
fprintf('✓ Particle Filter initialized (100 particles)\n\n');

%% Storage Arrays
ekf_estimates = zeros(steps, 4);
ukf_estimates = zeros(steps, 4);
pf_estimates = zeros(steps, 4);

ekf_errors = zeros(steps, 1);
ukf_errors = zeros(steps, 1);
pf_errors = zeros(steps, 1);

%% Main Simulation Loop
fprintf('Running simulation...\n');
start_time = tic;

for i = 1:steps
    % True state
    true_state = [true_x(i); true_y(i); true_theta(i); true_v(i)];
    
    % Compute control input (from true trajectory)
    if i < steps
        v_cmd = true_v(i);
        omega_cmd = (true_theta(i+1) - true_theta(i)) / dt;
    else
        v_cmd = true_v(i);
        omega_cmd = 0;
    end
    
    % Generate noisy measurement
    meas_noise = mvnrnd(zeros(3,1), measurement_noise_cov)';
    z_meas = true_state(1:3) + meas_noise;
    
    % EKF (uses [a, delta] control input format)
    if i > 1
        a_ekf = (true_v(i) - ekf_estimates(i-1, 4)) / dt;
    else
        a_ekf = 0;
    end
    delta_ekf = atan(L * omega_cmd / max(v_cmd, 0.1));
    u_ekf = [a_ekf; delta_ekf];
    ekf = ekf.predict(u_ekf);
    [ekf, ekf_state] = ekf.update(z_meas);
    ekf_estimates(i, :) = ekf_state';
    ekf_errors(i) = norm(ekf_state(1:3) - true_state(1:3));
    
    % UKF (uses [v, omega] control input format)
    u_ukf = [v_cmd; omega_cmd];
    ukf = ukf.predict(u_ukf);
    [ukf, ukf_state] = ukf.update(z_meas);
    ukf_estimates(i, :) = ukf_state';
    ukf_errors(i) = norm(ukf_state(1:3) - true_state(1:3));
    
    % Particle Filter (uses [v, omega] control input format)
    u_pf = [v_cmd; omega_cmd];
    pf = pf.predict(u_pf);
    pf = pf.update(z_meas);
    pf_state = pf.get_state_estimate();
    pf_estimates(i, :) = pf_state';
    pf_errors(i) = norm(pf_state(1:3) - true_state(1:3));
    
    % Progress update
    if mod(i, 50) == 0
        fprintf('  Step %d/%d (%.1f%%)\n', i, steps, 100*i/steps);
    end
end

elapsed_time = toc(start_time);
fprintf('✓ Simulation complete (%.2f seconds)\n\n', elapsed_time);

%% Performance Metrics
fprintf('=========================================================\n');
fprintf('PERFORMANCE COMPARISON\n');
fprintf('=========================================================\n\n');

fprintf('Extended Kalman Filter (EKF):\n');
fprintf('  Mean position error: %.4f m\n', mean(ekf_errors));
fprintf('  RMS error: %.4f m\n', sqrt(mean(ekf_errors.^2)));
fprintf('  Max error: %.4f m\n\n', max(ekf_errors));

fprintf('Unscented Kalman Filter (UKF):\n');
fprintf('  Mean position error: %.4f m\n', mean(ukf_errors));
fprintf('  RMS error: %.4f m\n', sqrt(mean(ukf_errors.^2)));
fprintf('  Max error: %.4f m\n\n', max(ukf_errors));

fprintf('Particle Filter (PF):\n');
fprintf('  Mean position error: %.4f m\n', mean(pf_errors));
fprintf('  RMS error: %.4f m\n', sqrt(mean(pf_errors.^2)));
fprintf('  Max error: %.4f m\n\n', max(pf_errors));

%% Visualization
fprintf('=========================================================\n');
fprintf('Generating visualizations...\n');
fprintf('=========================================================\n');

figure('Name', 'State Estimator Comparison', 'Position', [100, 100, 1400, 900]);

% 2D Trajectory
subplot(2,3,1);
plot(true_x, true_y, 'k-', 'DisplayName', 'True');
hold on;
plot(ekf_estimates(:,1), ekf_estimates(:,2), 'r--', 'DisplayName', 'EKF');
plot(ukf_estimates(:,1), ukf_estimates(:,2), 'b--','DisplayName', 'UKF');
plot(pf_estimates(:,1), pf_estimates(:,2), 'g--', 'DisplayName', 'PF');
xlabel('X (m)'); ylabel('Y (m)');
title('2D Trajectory Comparison');
legend('Location', 'best');
grid on; axis equal;

% Position Errors
subplot(2,3,2);
plot(t, ekf_errors, 'r-', 'DisplayName', 'EKF');
hold on;
plot(t, ukf_errors, 'b-', 'DisplayName', 'UKF');
plot(t, pf_errors, 'g-', 'DisplayName', 'PF');
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Estimation Errors');
legend('Location', 'best');
grid on;

% X Position
subplot(2,3,3);
plot(t, true_x, 'k-', 'DisplayName', 'True');
hold on;
plot(t, ekf_estimates(:,1), 'r--', 'DisplayName', 'EKF');
plot(t, ukf_estimates(:,1), 'b--', 'DisplayName', 'UKF');
plot(t, pf_estimates(:,1), 'g--', 'DisplayName', 'PF');
xlabel('Time (s)'); ylabel('X Position (m)');
title('X Position Tracking');
legend('Location', 'best');
grid on;

% Y Position
subplot(2,3,4);
plot(t, true_y, 'k-', 'LineWidth', 2, 'DisplayName', 'True');
hold on;
plot(t, ekf_estimates(:,2), 'r--', 'DisplayName', 'EKF');
plot(t, ukf_estimates(:,2), 'b--', 'DisplayName', 'UKF');
plot(t, pf_estimates(:,2), 'g--', 'DisplayName', 'PF');
xlabel('Time (s)'); ylabel('Y Position (m)');
title('Y Position Tracking');
legend('Location', 'best');
grid on;

% Theta
subplot(2,3,5);
plot(t, true_theta, 'k-', 'DisplayName', 'True');
hold on;
plot(t, ekf_estimates(:,3), 'r--', 'DisplayName', 'EKF');
plot(t, ukf_estimates(:,3), 'b--', 'DisplayName', 'UKF');
plot(t, pf_estimates(:,3), 'g--', 'DisplayName', 'PF');
xlabel('Time (s)'); ylabel('Heading (rad)');
title('Heading Tracking');
legend('Location', 'best');
grid on;

% Velocity
subplot(2,3,6);
plot(t, true_v, 'k-', 'DisplayName', 'True');
hold on;
plot(t, ekf_estimates(:,4), 'r--', 'DisplayName', 'EKF');
plot(t, ukf_estimates(:,4), 'b--', 'DisplayName', 'UKF');
plot(t, pf_estimates(:,4), 'g--', 'DisplayName', 'PF');
xlabel('Time (s)'); ylabel('Velocity (m/s)');
title('Velocity Tracking');
legend('Location', 'best');
grid on;

fprintf('✓ Visualization complete\n\n');

fprintf('=========================================================\n');
fprintf('TEST COMPLETE\n');
fprintf('=========================================================\n');

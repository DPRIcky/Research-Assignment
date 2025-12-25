classdef UKFEstimator
    % UKFEstimator - Unscented Kalman Filter for state estimation
    % Uses unscented transform for nonlinear systems
    
    properties
        dt                % Time step
        state             % Current state estimate [x; y; theta; v]
        P                 % State covariance matrix
        Q                 % Process noise covariance
        R                 % Measurement noise covariance
        alpha             % UKF spread parameter (default: 1e-3)
        beta              % UKF parameter for distribution (default: 2 for Gaussian)
        kappa             % UKF parameter (default: 0)
        lambda            % Composite scaling parameter
        gamma             % Sqrt of (n + lambda)
        Wm                % Weights for mean
        Wc                % Weights for covariance
        L                 % Vehicle wheelbase
    end
    
    methods
        function obj = UKFEstimator(dt, process_noise, measurement_noise, L)
            % Constructor
            % dt: time step
            % process_noise: 4x4 process noise covariance matrix
            % measurement_noise: 3x3 measurement noise covariance matrix
            % L: vehicle wheelbase (default: 2.5)
            
            obj.dt = dt;
            obj.Q = process_noise;
            obj.R = measurement_noise;
            
            if nargin < 4
                obj.L = 2.5;
            else
                obj.L = L;
            end
            
            % Initialize state and covariance (will be set in initialize)
            obj.state = zeros(4, 1);
            obj.P = eye(4) * 0.1;
            
            % UKF parameters
            obj.alpha = 1e-3;
            obj.beta = 2;
            obj.kappa = 0;
            
            n = 4;  % State dimension
            obj.lambda = obj.alpha^2 * (n + obj.kappa) - n;
            obj.gamma = sqrt(n + obj.lambda);
            
            % Weights
            obj.Wm = zeros(2*n + 1, 1);
            obj.Wc = zeros(2*n + 1, 1);
            
            obj.Wm(1) = obj.lambda / (n + obj.lambda);
            obj.Wc(1) = obj.lambda / (n + obj.lambda) + (1 - obj.alpha^2 + obj.beta);
            
            for i = 2:(2*n + 1)
                obj.Wm(i) = 1 / (2 * (n + obj.lambda));
                obj.Wc(i) = 1 / (2 * (n + obj.lambda));
            end
        end
        
        function obj = initialize(obj, initial_state)
            % Initialize filter with initial state
            obj.state = initial_state(:);
            obj.P = eye(4) * 0.1;
        end
        
        function [sigma_points] = generate_sigma_points(obj)
            % Generate sigma points using unscented transform
            n = length(obj.state);
            sigma_points = zeros(n, 2*n + 1);
            
            % Calculate matrix square root
            sqrt_P = chol((n + obj.lambda) * obj.P, 'lower');
            
            % First sigma point is the mean
            sigma_points(:, 1) = obj.state;
            
            % Generate remaining sigma points
            for i = 1:n
                sigma_points(:, i+1) = obj.state + sqrt_P(:, i);
                sigma_points(:, i+1+n) = obj.state - sqrt_P(:, i);
            end
        end
        
        function [x_new] = process_model(obj, x, u)
            % Nonlinear process model: bicycle kinematics
            % x: state [x, y, theta, v]
            % u: control [v_cmd, omega_cmd]
            
            v = u(1);
            omega = u(2);
            
            % Bicycle model
            x_new = zeros(4, 1);
            x_new(1) = x(1) + v * cos(x(3)) * obj.dt;
            x_new(2) = x(2) + v * sin(x(3)) * obj.dt;
            x_new(3) = x(3) + omega * obj.dt;
            x_new(4) = v;
            
            % Normalize angle to [-pi, pi]
            x_new(3) = atan2(sin(x_new(3)), cos(x_new(3)));
        end
        
        function [z] = measurement_model(~, x)
            % Measurement model: directly observe [x, y, theta]
            z = x(1:3);
        end
        
        function obj = predict(obj, u)
            % Prediction step
            
            % Generate sigma points
            sigma_points = obj.generate_sigma_points();
            n = size(sigma_points, 1);
            m = size(sigma_points, 2);
            
            % Propagate sigma points through process model
            sigma_points_pred = zeros(n, m);
            for i = 1:m
                sigma_points_pred(:, i) = obj.process_model(sigma_points(:, i), u);
            end
            
            % Compute predicted mean
            x_pred = zeros(n, 1);
            for i = 1:m
                x_pred = x_pred + obj.Wm(i) * sigma_points_pred(:, i);
            end
            
            % Handle angle wrapping for theta
            angles = sigma_points_pred(3, :);
            mean_angle = atan2(sum(obj.Wm .* sin(angles')), sum(obj.Wm .* cos(angles')));
            x_pred(3) = mean_angle;
            
            % Compute predicted covariance
            P_pred = obj.Q;
            for i = 1:m
                diff = sigma_points_pred(:, i) - x_pred;
                % Angle wrapping for theta difference
                diff(3) = atan2(sin(diff(3)), cos(diff(3)));
                P_pred = P_pred + obj.Wc(i) * (diff * diff');
            end
            
            obj.state = x_pred;
            obj.P = P_pred;
        end
        
        function [obj, state_out] = update(obj, z_meas)
            % Update step with measurement
            
            % Generate sigma points from predicted state
            sigma_points = obj.generate_sigma_points();
            n = size(sigma_points, 1);
            m = size(sigma_points, 2);
            nz = length(z_meas);
            
            % Propagate sigma points through measurement model
            Z = zeros(nz, m);
            for i = 1:m
                Z(:, i) = obj.measurement_model(sigma_points(:, i));
            end
            
            % Compute predicted measurement mean
            z_pred = zeros(nz, 1);
            for i = 1:m
                z_pred = z_pred + obj.Wm(i) * Z(:, i);
            end
            
            % Handle angle wrapping for measured theta
            angles_meas = Z(3, :);
            mean_angle_meas = atan2(sum(obj.Wm .* sin(angles_meas')), sum(obj.Wm .* cos(angles_meas')));
            z_pred(3) = mean_angle_meas;
            
            % Compute innovation covariance
            Pzz = obj.R;
            for i = 1:m
                diff_z = Z(:, i) - z_pred;
                diff_z(3) = atan2(sin(diff_z(3)), cos(diff_z(3)));
                Pzz = Pzz + obj.Wc(i) * (diff_z * diff_z');
            end
            
            % Compute cross-covariance
            Pxz = zeros(n, nz);
            for i = 1:m
                diff_x = sigma_points(:, i) - obj.state;
                diff_x(3) = atan2(sin(diff_x(3)), cos(diff_x(3)));
                diff_z = Z(:, i) - z_pred;
                diff_z(3) = atan2(sin(diff_z(3)), cos(diff_z(3)));
                Pxz = Pxz + obj.Wc(i) * (diff_x * diff_z');
            end
            
            % Kalman gain
            K = Pxz / Pzz;
            
            % Innovation
            innov = z_meas - z_pred;
            innov(3) = atan2(sin(innov(3)), cos(innov(3)));
            
            % Update state and covariance
            obj.state = obj.state + K * innov;
            obj.state(3) = atan2(sin(obj.state(3)), cos(obj.state(3)));
            obj.P = obj.P - K * Pzz * K';
            
            state_out = obj.state;
        end
    end
end

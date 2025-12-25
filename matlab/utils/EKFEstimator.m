classdef EKFEstimator
    % Extended Kalman Filter for bicycle model state estimation
    % State: [x; y; theta; v]
    % Measurement: [x; y; theta]
    
    properties
        dt              % Time step
        Q               % Process noise covariance (4x4)
        R               % Measurement noise covariance (3x3)
        x_hat           % State estimate (4x1)
        P               % State covariance (4x4)
        L               % Bicycle wheelbase (m)
    end
    
    methods
        function obj = EKFEstimator(dt, Q, R, L)
            % Constructor
            % dt: time step
            % Q: process noise covariance matrix (4x4)
            % R: measurement noise covariance matrix (3x3)
            % L: wheelbase (optional, default 2.5m)
            
            obj.dt = dt;
            obj.Q = Q;
            obj.R = R;
            
            if nargin < 4
                obj.L = 2.5;  % Default wheelbase
            else
                obj.L = L;
            end
            
            % Initialize with zero state and high uncertainty
            obj.x_hat = zeros(4, 1);
            obj.P = eye(4) * 10;  % High initial uncertainty
        end
        
        function obj = initialize(obj, initial_state)
            % Initialize filter with known initial state
            % initial_state: [x, y, theta, v] or [x; y; theta; v]
            
            obj.x_hat = initial_state(:);  % Ensure column vector
            obj.P = eye(4) * 0.1;  % Low initial uncertainty
        end
        
        function obj = predict(obj, u)
            % EKF prediction step
            % u: control input [a; delta] (acceleration and steering angle)
            
            % Extract state
            x = obj.x_hat(1);
            y = obj.x_hat(2);
            theta = obj.x_hat(3);
            v = obj.x_hat(4);
            
            if nargin < 2 || isempty(u)
                u = [0; 0];
            end
            
            a = u(1);  % Acceleration
            delta = u(2);  % Steering angle
            
            % Bicycle model dynamics: x_{k+1} = f(x_k, u_k)
            % x_dot = v * cos(theta)
            % y_dot = v * sin(theta)
            % theta_dot = v / L * tan(delta)
            % v_dot = a
            
            % Predict next state using nonlinear model
            x_new = x + obj.dt * v * cos(theta);
            y_new = y + obj.dt * v * sin(theta);
            theta_new = theta + obj.dt * v / obj.L * tan(delta);
            v_new = v + obj.dt * a;
            
            % Normalize angle
            theta_new = atan2(sin(theta_new), cos(theta_new));
            
            % Clamp velocity
            v_new = max(0, min(v_new, 5.0));
            
            % Compute Jacobian of dynamics F = df/dx
            % F is 4x4 matrix of partial derivatives
            F = eye(4);
            F(1, 3) = -obj.dt * v * sin(theta);  % dx/dtheta
            F(1, 4) = obj.dt * cos(theta);        % dx/dv
            F(2, 3) = obj.dt * v * cos(theta);    % dy/dtheta
            F(2, 4) = obj.dt * sin(theta);        % dy/dv
            F(3, 4) = obj.dt / obj.L * tan(delta); % dtheta/dv
            
            % Predict covariance: P = F*P*F' + Q
            obj.P = F * obj.P * F' + obj.Q;
            
            % Update state estimate
            obj.x_hat = [x_new; y_new; theta_new; v_new];
        end
        
        function [obj, state_estimate] = update(obj, z)
            % EKF update step with measurement
            % z: measurement vector [x_meas; y_meas; theta_meas] (3x1)
            
            if isempty(z)
                state_estimate = obj.x_hat;
                return;
            end
            
            % Ensure column vector
            z = z(:);
            
            % Measurement model: z = H * x + v
            % We measure [x, y, theta] directly
            H = [1, 0, 0, 0;
                 0, 1, 0, 0;
                 0, 0, 1, 0];
            
            % Predicted measurement
            z_pred = H * obj.x_hat;
            
            % Measurement residual (innovation)
            y = z - z_pred;
            
            % Normalize angle difference
            y(3) = atan2(sin(y(3)), cos(y(3)));
            
            % Innovation covariance
            S = H * obj.P * H' + obj.R;
            
            % Kalman gain
            K = obj.P * H' / S;
            
            % Update state estimate
            obj.x_hat = obj.x_hat + K * y;
            
            % Normalize heading in state
            obj.x_hat(3) = atan2(sin(obj.x_hat(3)), cos(obj.x_hat(3)));
            
            % Update covariance
            obj.P = (eye(4) - K * H) * obj.P;
            
            % Return updated state as row vector for compatibility
            state_estimate = obj.x_hat';
        end
        
        function state = get_state(obj)
            % Get current state estimate
            % Returns row vector [x, y, theta, v]
            state = obj.x_hat';
        end
        
        function cov = get_covariance(obj)
            % Get current state covariance matrix
            cov = obj.P;
        end
        
        function uncertainty = get_position_uncertainty(obj)
            % Get position uncertainty (2-sigma bound)
            % Returns [sigma_x, sigma_y] in meters
            uncertainty = 2 * sqrt([obj.P(1,1), obj.P(2,2)]);
        end
    end
end

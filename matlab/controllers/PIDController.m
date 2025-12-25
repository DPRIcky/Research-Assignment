classdef PIDController
    % PIDController - PID trajectory tracking controller for vehicle
    
    properties
        Kp_v        % Proportional gain for velocity
        Ki_v        % Integral gain for velocity
        Kd_v        % Derivative gain for velocity
        Kp_w        % Proportional gain for angular velocity
        Ki_w        % Integral gain for angular velocity
        Kd_w        % Derivative gain for angular velocity
        
        integral_v  % Integral error for velocity
        integral_w  % Integral error for angular velocity
        prev_error_v % Previous velocity error
        prev_error_w % Previous angular error
        
        v_max       % Maximum linear velocity (m/s)
        w_max       % Maximum angular velocity (rad/s)
        a_max       % Maximum linear acceleration (m/s^2)
        alpha_max   % Maximum angular acceleration (rad/s^2)
    end
    
    methods
        function obj = PIDController(Kp_v, Ki_v, Kd_v, Kp_w, Ki_w, Kd_w)
            % Constructor with PID gains
            if nargin < 6
                % Default gains
                Kp_v = 1.0;
                Ki_v = 0.1;
                Kd_v = 0.5;
                Kp_w = 2.0;
                Ki_w = 0.1;
                Kd_w = 0.3;
            end
            
            obj.Kp_v = Kp_v;
            obj.Ki_v = Ki_v;
            obj.Kd_v = Kd_v;
            obj.Kp_w = Kp_w;
            obj.Ki_w = Ki_w;
            obj.Kd_w = Kd_w;
            
            obj.integral_v = 0;
            obj.integral_w = 0;
            obj.prev_error_v = 0;
            obj.prev_error_w = 0;
            
            % Velocity and acceleration limits
            obj.v_max = 5.0;      % m/s
            obj.w_max = pi/2;     % rad/s
            obj.a_max = 2.0;      % m/s^2
            obj.alpha_max = pi;   % rad/s^2
        end
        
        function obj = reset(obj)
            % Reset integral and derivative terms
            obj.integral_v = 0;
            obj.integral_w = 0;
            obj.prev_error_v = 0;
            obj.prev_error_w = 0;
        end
        
        function [v_cmd, w_cmd, obj] = compute_control(obj, current_state, desired_state, dt)
            % Compute control commands using PID
            % current_state: [x, y, theta, v] or [x, y, theta]
            % desired_state: [x_d, y_d, theta_d, v_d]
            % dt: time step
            
            % Extract current state
            x = current_state(1);
            y = current_state(2);
            theta = current_state(3);
            if length(current_state) >= 4
                v_current = current_state(4);
            else
                v_current = 0;
            end
            
            % Extract desired state
            x_d = desired_state(1);
            y_d = desired_state(2);
            theta_d = desired_state(3);
            if length(desired_state) >= 4
                v_d = desired_state(4);
            else
                v_d = 1.0; % Default desired velocity
            end
            
            % Position error in global frame
            dx = x_d - x;
            dy = y_d - y;
            
            % Transform to local frame
            e_x = cos(theta) * dx + sin(theta) * dy;
            e_y = -sin(theta) * dx + cos(theta) * dy;
            
            % Distance to target
            dist = sqrt(dx^2 + dy^2);
            
            % Heading error (normalized to [-pi, pi])
            e_theta = atan2(sin(theta_d - theta), cos(theta_d - theta));
            
            % Desired velocity based on distance
            if dist > 0.1
                v_desired = min(v_d, dist);
            else
                v_desired = 0;
            end
            
            % Velocity error
            error_v = v_desired - v_current;
            
            % Update integral
            obj.integral_v = obj.integral_v + error_v * dt;
            obj.integral_w = obj.integral_w + e_theta * dt;
            
            % Anti-windup
            obj.integral_v = max(min(obj.integral_v, 1.0), -1.0);
            obj.integral_w = max(min(obj.integral_w, 1.0), -1.0);
            
            % Derivative
            deriv_v = (error_v - obj.prev_error_v) / dt;
            deriv_w = (e_theta - obj.prev_error_w) / dt;
            
            % PID control
            v_cmd = obj.Kp_v * error_v + obj.Ki_v * obj.integral_v + obj.Kd_v * deriv_v;
            w_cmd = obj.Kp_w * e_theta + obj.Ki_w * obj.integral_w + obj.Kd_w * deriv_w;
            
            % Add feedforward
            v_cmd = v_cmd + v_desired;
            
            % Apply velocity limits
            v_cmd = max(min(v_cmd, obj.v_max), 0);
            w_cmd = max(min(w_cmd, obj.w_max), -obj.w_max);
            
            % Store errors for next iteration
            obj.prev_error_v = error_v;
            obj.prev_error_w = e_theta;
        end
        
        function [v_cmd, w_cmd, obj] = track_path(obj, current_state, path, lookahead_dist, dt)
            % Track a path using pure pursuit + PID
            % current_state: [x, y, theta, v]
            % path: Nx3 array [x, y, theta]
            % lookahead_dist: lookahead distance for pure pursuit
            % dt: time step
            
            x = current_state(1);
            y = current_state(2);
            theta = current_state(3);
            if length(current_state) >= 4
                v_current = current_state(4);
            else
                v_current = 0;
            end
            
            % Find closest point on path
            distances = sqrt((path(:,1) - x).^2 + (path(:,2) - y).^2);
            [~, closest_idx] = min(distances);
            
            % Find lookahead point
            lookahead_idx = closest_idx;
            for i = closest_idx:size(path, 1)
                dist = sqrt((path(i,1) - x)^2 + (path(i,2) - y)^2);
                if dist >= lookahead_dist
                    lookahead_idx = i;
                    break;
                end
            end
            
            % If at end of path, use last point
            if lookahead_idx >= size(path, 1)
                lookahead_idx = size(path, 1);
            end
            
            % Desired state from lookahead point
            desired_state = [path(lookahead_idx, 1), ...
                           path(lookahead_idx, 2), ...
                           path(lookahead_idx, 3), ...
                           1.0]; % desired velocity
            
            % Compute control
            [v_cmd, w_cmd, obj] = obj.compute_control(current_state, desired_state, dt);
        end
        
        function stats = get_tracking_error(~, actual_traj, desired_path)
            % Calculate tracking errors
            % actual_traj: Mx3 [x, y, theta]
            % desired_path: Nx3 [x, y, theta]
            
            % Find closest point for each trajectory point
            errors = zeros(size(actual_traj, 1), 1);
            
            for i = 1:size(actual_traj, 1)
                distances = sqrt((desired_path(:,1) - actual_traj(i,1)).^2 + ...
                               (desired_path(:,2) - actual_traj(i,2)).^2);
                errors(i) = min(distances);
            end
            
            stats = struct('mean_error', mean(errors), ...
                          'max_error', max(errors), ...
                          'rms_error', sqrt(mean(errors.^2)), ...
                          'final_error', errors(end));
        end
    end
end

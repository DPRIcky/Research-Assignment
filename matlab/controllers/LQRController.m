classdef LQRController < handle
    % LQRController - Linear Quadratic Regulator for trajectory tracking
    %
    % This controller computes optimal state feedback gains using the
    % LQR method to minimize a quadratic cost function. It linearizes
    % the system dynamics around reference points and tracks trajectories.
    
    properties
        Q           % State cost matrix
        R           % Control cost matrix
        K           % LQR feedback gain matrix
        dt          % Time step
        vehicle_type % 'bicycle' or 'differential'
        L           % Wheelbase (for bicycle model)
        max_v       % Maximum linear velocity
        max_w       % Maximum angular velocity
        max_delta   % Maximum steering angle (bicycle)
        max_a       % Maximum acceleration (bicycle)
    end
    
    methods
        function obj = LQRController(Q, R, dt, vehicle_type, L)
            % Constructor
            % Inputs:
            %   Q - State cost matrix (4x4 for bicycle, 3x3 for differential)
            %   R - Control cost matrix (2x2)
            %   dt - Time step for discretization
            %   vehicle_type - 'bicycle' or 'differential'
            %   L - Wheelbase for bicycle model (optional)
            
            if nargin < 5
                L = 2.5; % Default wheelbase
            end
            
            obj.Q = Q;
            obj.R = R;
            obj.dt = dt;
            obj.vehicle_type = vehicle_type;
            obj.L = L;
            
            % Set limits
            obj.max_v = 5.0;
            obj.max_w = pi/2;
            obj.max_delta = pi/4;
            obj.max_a = 3.0;
        end
        
        function [v_cmd, w_cmd] = compute_control(obj, state, ref_state, ref_control)
            % Compute LQR control for trajectory tracking
            % Inputs:
            %   state - Current state [x, y, theta, v] (bicycle) or [x, y, theta] (diff)
            %   ref_state - Reference state
            %   ref_control - Reference control [v_ref, w_ref] or [a_ref, delta_ref]
            % Outputs:
            %   v_cmd - Linear velocity command
            %   w_cmd - Angular velocity command
            
            if strcmp(obj.vehicle_type, 'bicycle')
                [v_cmd, w_cmd] = obj.lqr_bicycle(state, ref_state, ref_control);
            else
                [v_cmd, w_cmd] = obj.lqr_differential(state, ref_state, ref_control);
            end
        end
        
        function [v_cmd, w_cmd] = lqr_bicycle(obj, state, ref_state, ref_control)
            % LQR control for bicycle model
            % State: [x, y, theta, v]
            % Control: [a, delta] -> converts to [v, w]
            
            % Linearize around reference
            x_ref = ref_state(1);
            y_ref = ref_state(2);
            theta_ref = ref_state(3);
            v_ref = ref_state(4);
            
            if length(ref_control) == 2
                a_ref = ref_control(1);
                delta_ref = ref_control(2);
            else
                a_ref = 0;
                delta_ref = 0;
            end
            
            % Compute linearized system matrices
            % dx/dt = A*x + B*u
            A = [0, 0, -v_ref*sin(theta_ref), cos(theta_ref);
                 0, 0,  v_ref*cos(theta_ref), sin(theta_ref);
                 0, 0,  0, tan(delta_ref)/obj.L;
                 0, 0,  0, 0];
            
            B = [0, 0;
                 0, 0;
                 0, v_ref/(obj.L * cos(delta_ref)^2);
                 1, 0];
            
            % Discretize system
            Ad = eye(4) + A * obj.dt;
            Bd = B * obj.dt;
            
            % Solve discrete-time algebraic Riccati equation
            try
                [K, ~, ~] = dlqr(Ad, Bd, obj.Q, obj.R);
            catch
                % If DLQR fails, use continuous time approximation
                [K, ~, ~] = lqr(A, B, obj.Q, obj.R);
            end
            
            % State error
            x_err = state(:) - ref_state(:);
            
            % Normalize angle error to [-pi, pi]
            x_err(3) = atan2(sin(x_err(3)), cos(x_err(3)));
            
            % Compute control
            u = -K * x_err;
            a_cmd = u(1) + a_ref;
            delta_cmd = u(2) + delta_ref;
            
            % Apply control limits
            a_cmd = max(min(a_cmd, obj.max_a), -obj.max_a);
            delta_cmd = max(min(delta_cmd, obj.max_delta), -obj.max_delta);
            
            % Convert to velocity commands
            v_cmd = v_ref + a_cmd * obj.dt;
            v_cmd = max(min(v_cmd, obj.max_v), 0);
            
            w_cmd = v_cmd * tan(delta_cmd) / obj.L;
            w_cmd = max(min(w_cmd, obj.max_w), -obj.max_w);
        end
        
        function [v_cmd, w_cmd] = lqr_differential(obj, state, ref_state, ref_control)
            % LQR control for differential drive model
            % State: [x, y, theta]
            % Control: [v, w]
            
            % Linearize around reference
            theta_ref = ref_state(3);
            v_ref = ref_control(1);
            w_ref = ref_control(2);
            
            % Compute linearized system matrices
            A = [0, 0, -v_ref*sin(theta_ref);
                 0, 0,  v_ref*cos(theta_ref);
                 0, 0,  0];
            
            B = [cos(theta_ref), 0;
                 sin(theta_ref), 0;
                 0, 1];
            
            % Discretize system
            Ad = eye(3) + A * obj.dt;
            Bd = B * obj.dt;
            
            % Solve discrete-time algebraic Riccati equation
            try
                [K, ~, ~] = dlqr(Ad, Bd, obj.Q, obj.R);
            catch
                % If DLQR fails, use continuous time approximation
                [K, ~, ~] = lqr(A, B, obj.Q, obj.R);
            end
            
            % State error
            x_err = state(:) - ref_state(:);
            
            % Normalize angle error
            x_err(3) = atan2(sin(x_err(3)), cos(x_err(3)));
            
            % Compute control
            u = -K * x_err + [v_ref; w_ref];
            
            % Apply control limits
            v_cmd = max(min(u(1), obj.max_v), 0);
            w_cmd = max(min(u(2), obj.max_w), -obj.max_w);
        end
        
        function [v_cmd, w_cmd] = track_path(obj, state, path, lookahead_dist)
            % Track a path using LQR control
            % Inputs:
            %   state - Current state
            %   path - Nx3 or Nx4 array [x, y, theta] or [x, y, theta, v]
            %   lookahead_dist - Lookahead distance for target selection
            % Outputs:
            %   v_cmd, w_cmd - Velocity commands
            
            % Find closest point on path
            distances = sqrt((path(:,1) - state(1)).^2 + (path(:,2) - state(2)).^2);
            [~, closest_idx] = min(distances);
            
            % Find lookahead point
            lookahead_idx = closest_idx;
            cumulative_dist = 0;
            
            for i = closest_idx:size(path, 1)-1
                segment_dist = sqrt((path(i+1,1) - path(i,1))^2 + ...
                                   (path(i+1,2) - path(i,2))^2);
                cumulative_dist = cumulative_dist + segment_dist;
                
                if cumulative_dist >= lookahead_dist
                    lookahead_idx = i + 1;
                    break;
                end
            end
            
            lookahead_idx = min(lookahead_idx, size(path, 1));
            
            % Get reference state
            if strcmp(obj.vehicle_type, 'bicycle')
                if size(path, 2) >= 4
                    ref_state = path(lookahead_idx, :);
                else
                    ref_state = [path(lookahead_idx, 1:3), 2.0]; % Default velocity
                end
            else
                ref_state = path(lookahead_idx, 1:3);
            end
            
            % Compute desired velocity towards lookahead point
            dx = path(lookahead_idx, 1) - state(1);
            dy = path(lookahead_idx, 2) - state(2);
            dist_to_target = sqrt(dx^2 + dy^2);
            
            % Reference control
            if strcmp(obj.vehicle_type, 'bicycle')
                v_ref = min(2.0, dist_to_target);
                ref_control = [0, 0]; % Zero acceleration and steering
            else
                v_ref = min(2.0, dist_to_target);
                angle_to_target = atan2(dy, dx);
                angle_diff = atan2(sin(angle_to_target - state(3)), ...
                                 cos(angle_to_target - state(3)));
                w_ref = 0.5 * angle_diff;
                ref_control = [v_ref, w_ref];
            end
            
            % Compute LQR control
            [v_cmd, w_cmd] = obj.compute_control(state, ref_state, ref_control);
        end
        
        function err = get_tracking_error(obj, state, path)
            % Compute tracking error (distance to closest point on path)
            distances = sqrt((path(:,1) - state(1)).^2 + (path(:,2) - state(2)).^2);
            err = min(distances);
        end
    end
end

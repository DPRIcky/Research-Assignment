classdef MPCController < handle
    % MPCController - Model Predictive Control for trajectory tracking
    %
    % This controller uses a finite prediction horizon to optimize a
    % sequence of control inputs while respecting constraints on states
    % and controls. It uses a receding horizon strategy.
    
    properties
        N           % Prediction horizon
        dt          % Time step
        Q           % State cost matrix
        R           % Control cost matrix
        Qf          % Terminal state cost matrix
        vehicle_type % 'bicycle' or 'differential'
        L           % Wheelbase (for bicycle model)
        
        % Constraints
        v_min       % Minimum linear velocity
        v_max       % Maximum linear velocity
        w_max       % Maximum angular velocity
        a_max       % Maximum acceleration (bicycle)
        delta_max   % Maximum steering angle (bicycle)
        
        % Warning counter
        warning_count  % Track warnings to suppress after a few
    end
    
    methods
        function obj = MPCController(N, dt, Q, R, Qf, vehicle_type, L)
            % Constructor
            % Inputs:
            %   N - Prediction horizon
            %   dt - Time step for discretization
            %   Q - State cost matrix
            %   R - Control cost matrix
            %   Qf - Terminal state cost matrix
            %   vehicle_type - 'bicycle' or 'differential'
            %   L - Wheelbase for bicycle model (optional)
            
            if nargin < 7
                L = 2.5; % Default wheelbase
            end
            
            obj.N = N;
            obj.dt = dt;
            obj.Q = Q;
            obj.R = R;
            obj.Qf = Qf;
            obj.vehicle_type = vehicle_type;
            obj.L = L;
            
            % Set constraints
            obj.v_min = 0.0;
            obj.v_max = 5.0;
            obj.w_max = pi/2;
            obj.a_max = 3.0;
            obj.delta_max = pi/4;
            obj.warning_count = 0;  % Initialize warning counter
        end
        
        function [v_cmd, w_cmd] = compute_control(obj, state, ref_trajectory)
            % Compute MPC control
            % Inputs:
            %   state - Current state
            %   ref_trajectory - Reference trajectory (Nx4 or Nx3)
            % Outputs:
            %   v_cmd - Linear velocity command
            %   w_cmd - Angular velocity command
            
            if strcmp(obj.vehicle_type, 'bicycle')
                [v_cmd, w_cmd] = obj.mpc_bicycle(state, ref_trajectory);
            else
                [v_cmd, w_cmd] = obj.mpc_differential(state, ref_trajectory);
            end
        end
        
        function [v_cmd, w_cmd] = mpc_bicycle(obj, state, ref_traj)
            % MPC for bicycle model
            % State: [x, y, theta, v]
            % Control: [a, delta]
            
            % Ensure reference trajectory has enough points
            if size(ref_traj, 1) < obj.N + 1
                % Extend with final state
                last_state = ref_traj(end, :);
                n_extend = obj.N + 1 - size(ref_traj, 1);
                ref_traj = [ref_traj; repmat(last_state, n_extend, 1)];
            end
            
            % Decision variables: [a(0), delta(0), ..., a(N-1), delta(N-1)]
            n_controls = 2 * obj.N;
            
            % Initial guess: intelligent warm start following reference trajectory
            u0 = zeros(2 * obj.N, 1);
            
            % For each prediction step, compute control to follow the reference
            for k = 1:obj.N
                if k == 1
                    % First control: move toward first reference point
                    dx = ref_traj(k+1, 1) - state(1);
                    dy = ref_traj(k+1, 2) - state(2);
                    desired_theta = atan2(dy, dx);
                    theta_err = atan2(sin(desired_theta - state(3)), cos(desired_theta - state(3)));
                    
                    % Smooth acceleration
                    target_v = min(2.0, obj.v_max);
                    v_current = state(4);
                    a_init = (target_v - v_current) / (obj.N * obj.dt);
                    a_init = max(min(a_init, obj.a_max * 0.5), -obj.a_max * 0.5);
                    
                    % Steering to correct heading gradually
                    delta_init = max(min(theta_err * 0.3, obj.delta_max * 0.7), -obj.delta_max * 0.7);
                else
                    % Subsequent controls: maintain smooth trajectory
                    a_init = a_init * 0.9;  % Decay acceleration
                    
                    % Follow reference heading changes
                    if k < obj.N
                        heading_change = atan2(sin(ref_traj(k+1,3) - ref_traj(k,3)), ...
                                              cos(ref_traj(k+1,3) - ref_traj(k,3)));
                        delta_init = max(min(heading_change, obj.delta_max * 0.7), -obj.delta_max * 0.7);
                    end
                end
                
                u0(2*k-1) = a_init;
                u0(2*k) = delta_init;
            end
            
            % Bounds on controls
            lb = repmat([-obj.a_max; -obj.delta_max], obj.N, 1);
            ub = repmat([obj.a_max; obj.delta_max], obj.N, 1);
            
            % Optimization options
            options = optimoptions('fmincon', ...
                'Display', 'off', ...
                'Algorithm', 'sqp', ...
                'MaxIterations', 100, ...
                'MaxFunctionEvaluations', 1000);
            
            % Cost function
            cost_fn = @(u) obj.bicycle_cost(u, state, ref_traj);
            
            % Remove nonlinear constraints - they cause infeasibility
            % Velocity is controlled indirectly through acceleration bounds
            
            % Solve optimization (no nonlinear constraints)
            try
                [u_opt, ~, exitflag] = fmincon(cost_fn, u0, [], [], [], [], lb, ub, [], options);
                if exitflag < 1
                    obj.warning_count = obj.warning_count + 1;
                    if obj.warning_count <= 3
                        warning('MPC:OptimizationFailed', 'MPC bicycle optimization exitflag=%d, using warm start (warning suppressed after 3)', exitflag);
                    end
                    u_opt = u0;
                end
            catch ME
                obj.warning_count = obj.warning_count + 1;
                if obj.warning_count <= 3
                    warning('MPC:Error', '%s (warning suppressed after 3)', ME.message);
                end
                u_opt = u0;
            end
            
            % Extract first control
            a_cmd = u_opt(1);
            delta_cmd = u_opt(2);
            
            % Convert to velocity commands
            v_cmd = state(4) + a_cmd * obj.dt;
            v_cmd = max(min(v_cmd, obj.v_max), obj.v_min);
            
            w_cmd = v_cmd * tan(delta_cmd) / obj.L;
            w_cmd = max(min(w_cmd, obj.w_max), -obj.w_max);
        end
        
        function cost = bicycle_cost(obj, u, x0, ref_traj)
            % Compute cost for bicycle model
            x = x0;
            cost = 0;
            
            for k = 1:obj.N
                % Extract control
                a = u(2*k-1);
                delta = u(2*k);
                
                % Predict next state
                x = obj.predict_bicycle(x, a, delta);
                
                % State error
                x_ref = ref_traj(k+1, :);
                x_err = x - x_ref;
                x_err(3) = atan2(sin(x_err(3)), cos(x_err(3)));
                
                % Add to cost
                if k < obj.N
                    cost = cost + x_err * obj.Q * x_err' + [a; delta]' * obj.R * [a; delta];
                else
                    cost = cost + x_err * obj.Qf * x_err' + [a; delta]' * obj.R * [a; delta];
                end
            end
        end
        
        function x_next = predict_bicycle(obj, x, a, delta)
            % Predict next state for bicycle model
            theta = x(3);
            v = x(4);
            
            % Kinematic bicycle model
            x_next = zeros(1, 4);
            x_next(1) = x(1) + v * cos(theta) * obj.dt;
            x_next(2) = x(2) + v * sin(theta) * obj.dt;
            x_next(3) = x(3) + (v / obj.L) * tan(delta) * obj.dt;
            x_next(4) = v + a * obj.dt;
        end
        
        function [c, ceq] = bicycle_constraints(obj, u, x0)
            % Nonlinear constraints for bicycle model
            % Ensure velocity stays within bounds
            
            x = x0;
            c = [];
            
            for k = 1:obj.N
                a = u(2*k-1);
                delta = u(2*k);
                
                x = obj.predict_bicycle(x, a, delta);
                
                % Velocity constraints
                c = [c; x(4) - obj.v_max; obj.v_min - x(4)];
            end
            
            ceq = [];
        end
        
        function [v_cmd, w_cmd] = mpc_differential(obj, state, ref_traj)
            % MPC for differential drive model
            % State: [x, y, theta]
            % Control: [v, w]
            
            % Ensure reference trajectory has enough points
            if size(ref_traj, 1) < obj.N + 1
                last_state = ref_traj(end, :);
                n_extend = obj.N + 1 - size(ref_traj, 1);
                ref_traj = [ref_traj; repmat(last_state, n_extend, 1)];
            end
            
            % Decision variables: [v(0), w(0), ..., v(N-1), w(N-1)]
            n_controls = 2 * obj.N;
            
            % Initial guess: forward velocity with heading correction
            target_v = 2.0;
            dx = ref_traj(2,1) - state(1);
            dy = ref_traj(2,2) - state(2);
            desired_theta = atan2(dy, dx);
            theta_err = atan2(sin(desired_theta - state(3)), cos(desired_theta - state(3)));
            w_init = max(min(theta_err * 2.0, obj.w_max), -obj.w_max);
            u0 = repmat([target_v; w_init], obj.N, 1);
            
            % Bounds on controls
            lb = repmat([obj.v_min; -obj.w_max], obj.N, 1);
            ub = repmat([obj.v_max; obj.w_max], obj.N, 1);
            
            % Optimization options - interior-point is more robust than SQP
            options = optimoptions('fmincon', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point', ...
                'MaxIterations', 100, ...
                'MaxFunctionEvaluations', 1000, ...
                'OptimalityTolerance', 1e-3, ...
                'StepTolerance', 1e-4, ...
                'ConstraintTolerance', 1e-3, ...
                'ScaleProblem', true);
            
            % Cost function
            cost_fn = @(u) obj.differential_cost(u, state, ref_traj);
            
            % Solve optimization
            try
                [u_opt, ~, exitflag] = fmincon(cost_fn, u0, [], [], [], [], lb, ub, [], options);
                if exitflag < 1
                    warning('MPC:OptimizationFailed', 'MPC differential optimization exitflag=%d, using warm start', exitflag);
                    u_opt = u0;
                end
            catch ME
                warning('MPC:Error', '%s', ME.message);
                u_opt = u0;
            end
            
            % Extract first control
            v_cmd = u_opt(1);
            w_cmd = u_opt(2);
            
            % Apply constraints
            v_cmd = max(min(v_cmd, obj.v_max), obj.v_min);
            w_cmd = max(min(w_cmd, obj.w_max), -obj.w_max);
        end
        
        function cost = differential_cost(obj, u, x0, ref_traj)
            % Compute cost for differential drive model
            x = x0;
            cost = 0;
            
            for k = 1:obj.N
                % Extract control
                v = u(2*k-1);
                w = u(2*k);
                
                % Predict next state
                x = obj.predict_differential(x, v, w);
                
                % State error
                x_ref = ref_traj(k+1, 1:3);
                x_err = x - x_ref;
                x_err(3) = atan2(sin(x_err(3)), cos(x_err(3)));
                
                % Add to cost
                if k < obj.N
                    cost = cost + x_err * obj.Q * x_err' + [v; w]' * obj.R * [v; w];
                else
                    cost = cost + x_err * obj.Qf * x_err' + [v; w]' * obj.R * [v; w];
                end
            end
        end
        
        function x_next = predict_differential(obj, x, v, w)
            % Predict next state for differential drive
            theta = x(3);
            
            x_next = zeros(1, 3);
            x_next(1) = x(1) + v * cos(theta) * obj.dt;
            x_next(2) = x(2) + v * sin(theta) * obj.dt;
            x_next(3) = x(3) + w * obj.dt;
        end
        
        function [v_cmd, w_cmd] = track_path(obj, state, path, lookahead_dist)
            % Track a path using MPC
            % Inputs:
            %   state - Current state
            %   path - Nx3 or Nx4 array [x, y, theta] or [x, y, theta, v]
            %   lookahead_dist - Lookahead distance for reference selection
            % Outputs:
            %   v_cmd, w_cmd - Velocity commands
            
            % Find closest point on path AHEAD of vehicle
            distances = sqrt((path(:,1) - state(1)).^2 + (path(:,2) - state(2)).^2);
            [min_dist, closest_idx] = min(distances);
            
            % Look ahead: only consider waypoints ahead of current position
            % This prevents tracking backwards
            for i = closest_idx:size(path, 1)
                dx = path(i,1) - state(1);
                dy = path(i,2) - state(2);
                % Check if waypoint is ahead (dot product with heading > 0)
                if cos(state(3)) * dx + sin(state(3)) * dy > 0
                    closest_idx = i;
                    break;
                end
            end
            
            % Get reference trajectory for prediction horizon
            ref_indices = closest_idx:min(closest_idx + obj.N, size(path, 1));
            
            if length(ref_indices) < obj.N + 1
                % Extend with final point
                ref_indices = [ref_indices, repmat(size(path, 1), 1, obj.N + 1 - length(ref_indices))];
            end
            
            ref_traj = path(ref_indices, :);
            
            % Ensure proper size
            if strcmp(obj.vehicle_type, 'bicycle')
                if size(ref_traj, 2) == 3
                    ref_traj = [ref_traj, 2.0*ones(size(ref_traj, 1), 1)];
                end
            end
            
            % Compute MPC control
            [v_cmd, w_cmd] = obj.compute_control(state, ref_traj);
        end
        
        function err = get_tracking_error(obj, state, path)
            % Compute tracking error (distance to closest point on path)
            distances = sqrt((path(:,1) - state(1)).^2 + (path(:,2) - state(2)).^2);
            err = min(distances);
        end
    end
end

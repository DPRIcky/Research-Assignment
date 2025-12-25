classdef CBFSafetyFilter < handle
    % CBFSafetyFilter - Control Barrier Function based safety filter
    %
    % This class implements a QP-based safety filter using Control Barrier
    % Functions (CBF) to ensure collision avoidance while minimally
    % modifying nominal controller outputs.
    
    properties
        env             % Environment with obstacles
        safety_margin   % Safety margin around obstacles (m)
        alpha           % CBF class-K function parameter
        vehicle_type    % 'bicycle' or 'differential'
        L               % Wheelbase (for bicycle model)
        dt              % Time step
        
        % Control limits
        v_min           % Minimum linear velocity
        v_max           % Maximum linear velocity
        w_max           % Maximum angular velocity
        a_max           % Maximum acceleration (bicycle)
        delta_max       % Maximum steering angle (bicycle)
        
        % QP solver options
        qp_options      % Options for quadprog
    end
    
    methods
        function obj = CBFSafetyFilter(env, safety_margin, alpha, vehicle_type, L, dt)
            % Constructor
            % Inputs:
            %   env - Environment object with obstacles
            %   safety_margin - Safety margin around obstacles (m)
            %   alpha - CBF class-K function parameter
            %   vehicle_type - 'bicycle' or 'differential'
            %   L - Wheelbase for bicycle model (optional)
            %   dt - Time step (optional)
            
            if nargin < 5
                L = 2.5;
            end
            if nargin < 6
                dt = 0.1;
            end
            
            obj.env = env;
            obj.safety_margin = safety_margin;
            obj.alpha = alpha;
            obj.vehicle_type = vehicle_type;
            obj.L = L;
            obj.dt = dt;
            
            % Set control limits
            obj.v_min = 0.0;
            obj.v_max = 5.0;
            obj.w_max = pi/2;
            obj.a_max = 3.0;
            obj.delta_max = pi/4;
            
            % QP solver options
            obj.qp_options = optimoptions('quadprog', ...
                'Display', 'off', ...
                'Algorithm', 'interior-point-convex');
        end
        
        function [v_safe, w_safe, is_modified] = filter_control(obj, state, v_nom, w_nom)
            % Apply CBF safety filter to nominal control
            % Inputs:
            %   state - Current state [x, y, theta, v] or [x, y, theta]
            %   v_nom - Nominal linear velocity command
            %   w_nom - Nominal angular velocity command
            % Outputs:
            %   v_safe - Safe linear velocity command
            %   w_safe - Safe angular velocity command
            %   is_modified - Boolean indicating if control was modified
            
            if strcmp(obj.vehicle_type, 'bicycle')
                [v_safe, w_safe, is_modified] = obj.filter_bicycle(state, v_nom, w_nom);
            else
                [v_safe, w_safe, is_modified] = obj.filter_differential(state, v_nom, w_nom);
            end
        end
        
        function [v_safe, w_safe, is_modified] = filter_bicycle(obj, state, v_nom, w_nom)
            % CBF filter for bicycle model
            % Control: [a, delta] (converted from v, w)
            
            % Convert nominal velocity commands to bicycle control
            v_current = state(4);
            a_nom = (v_nom - v_current) / obj.dt;
            delta_nom = atan(obj.L * w_nom / max(v_nom, 0.1));
            
            % Clamp to limits
            a_nom = max(min(a_nom, obj.a_max), -obj.a_max);
            delta_nom = max(min(delta_nom, obj.delta_max), -obj.delta_max);
            
            % Get CBF constraints
            [A_cbf, b_cbf] = obj.get_cbf_constraints_bicycle(state);
            
            if isempty(A_cbf)
                % No obstacles nearby, use nominal control
                v_safe = v_nom;
                w_safe = w_nom;
                is_modified = false;
                return;
            end
            
            % QP formulation: min ||u - u_nom||^2 subject to CBF constraints
            % Decision variable: u = [a; delta]
            H = eye(2);
            f = -[a_nom; delta_nom];
            
            % Control bounds
            lb = [-obj.a_max; -obj.delta_max];
            ub = [obj.a_max; obj.delta_max];
            
            % Solve QP
            try
                u_safe = quadprog(H, f, A_cbf, b_cbf, [], [], lb, ub, [], obj.qp_options);
                
                if isempty(u_safe)
                    % QP failed, use safest option (stop)
                    v_safe = 0;
                    w_safe = 0;
                    is_modified = true;
                    return;
                end
                
                a_safe = u_safe(1);
                delta_safe = u_safe(2);
                
                % Check if control was modified
                is_modified = (abs(a_safe - a_nom) > 1e-3) || (abs(delta_safe - delta_nom) > 1e-3);
                
                % Convert back to velocity commands
                v_safe = v_current + a_safe * obj.dt;
                v_safe = max(min(v_safe, obj.v_max), obj.v_min);
                w_safe = v_safe * tan(delta_safe) / obj.L;
                w_safe = max(min(w_safe, obj.w_max), -obj.w_max);
                
            catch
                % QP solver failed, stop for safety
                v_safe = 0;
                w_safe = 0;
                is_modified = true;
            end
        end
        
        function [v_safe, w_safe, is_modified] = filter_differential(obj, state, v_nom, w_nom)
            % CBF filter for differential drive model
            % Control: [v, w]
            
            % Get CBF constraints
            [A_cbf, b_cbf] = obj.get_cbf_constraints_differential(state);
            
            if isempty(A_cbf)
                % No obstacles nearby, use nominal control
                v_safe = v_nom;
                w_safe = w_nom;
                is_modified = false;
                return;
            end
            
            % QP formulation: min ||u - u_nom||^2 subject to CBF constraints
            % Decision variable: u = [v; w]
            H = eye(2);
            f = -[v_nom; w_nom];
            
            % Control bounds
            lb = [obj.v_min; -obj.w_max];
            ub = [obj.v_max; obj.w_max];
            
            % Solve QP
            try
                u_safe = quadprog(H, f, A_cbf, b_cbf, [], [], lb, ub, [], obj.qp_options);
                
                if isempty(u_safe)
                    v_safe = 0;
                    w_safe = 0;
                    is_modified = true;
                    return;
                end
                
                v_safe = u_safe(1);
                w_safe = u_safe(2);
                
                % Check if control was modified
                is_modified = (abs(v_safe - v_nom) > 1e-3) || (abs(w_safe - w_nom) > 1e-3);
                
            catch
                v_safe = 0;
                w_safe = 0;
                is_modified = true;
            end
        end
        
        function [A, b] = get_cbf_constraints_bicycle(obj, state)
            % Compute CBF constraints for bicycle model
            % Returns: A*u <= b where u = [a; delta]
            
            x = state(1);
            y = state(2);
            theta = state(3);
            v = state(4);
            
            A = [];
            b = [];
            
            % Check each obstacle
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                
                % Compute barrier function and its derivative
                [h, dh_dx, dh_dy] = obj.compute_barrier(x, y, obs);
                
                % Skip if obstacle is far away
                if h > 10.0
                    continue;
                end
                
                % CBF constraint: dh/dt + alpha*h >= 0
                % dh/dt = (dh/dx)*(dx/dt) + (dh/dy)*(dy/dt)
                %       = (dh/dx)*v*cos(theta) + (dh/dy)*v*sin(theta)
                
                % For bicycle model with control u = [a; delta]:
                % dx/dt = v*cos(theta)
                % dy/dt = v*sin(theta)
                % dtheta/dt = (v/L)*tan(delta)
                % dv/dt = a
                
                % Lie derivatives
                Lf_h = dh_dx * v * cos(theta) + dh_dy * v * sin(theta);
                
                % Control appears through v (from a) and theta (from delta)
                Lg_h = [dh_dx * cos(theta) + dh_dy * sin(theta), 0];
                
                % CBF constraint: Lf_h + Lg_h*u + alpha*h >= 0
                % Rearranged: -Lg_h*u <= Lf_h + alpha*h
                A = [A; -Lg_h];
                b = [b; Lf_h + obj.alpha * h];
            end
        end
        
        function [A, b] = get_cbf_constraints_differential(obj, state)
            % Compute CBF constraints for differential drive model
            % Returns: A*u <= b where u = [v; w]
            
            x = state(1);
            y = state(2);
            theta = state(3);
            
            A = [];
            b = [];
            
            % Check each obstacle
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                
                % Compute barrier function and its derivative
                [h, dh_dx, dh_dy] = obj.compute_barrier(x, y, obs);
                
                % Skip if obstacle is far away
                if h > 10.0
                    continue;
                end
                
                % For differential drive with control u = [v; w]:
                % dx/dt = v*cos(theta)
                % dy/dt = v*sin(theta)
                
                % Lie derivatives
                Lf_h = 0; % No drift dynamics
                Lg_h = [dh_dx * cos(theta) + dh_dy * sin(theta), 0];
                
                % CBF constraint: Lg_h*u + alpha*h >= 0
                A = [A; -Lg_h];
                b = [b; obj.alpha * h];
            end
        end
        
        function [h, dh_dx, dh_dy] = compute_barrier(obj, x, y, obstacle)
            % Compute barrier function value and gradient
            % h > 0 means safe, h = 0 means on boundary, h < 0 means collision
            
            if strcmp(obstacle.type, 'circle')
                % Distance to circle center
                dx = x - obstacle.x;
                dy = y - obstacle.y;
                dist = sqrt(dx^2 + dy^2);
                
                % Barrier function: h = dist - (radius + safety_margin)
                h = dist - (obstacle.radius + obj.safety_margin);
                
                % Gradient
                if dist > 1e-6
                    dh_dx = dx / dist;
                    dh_dy = dy / dist;
                else
                    % At center, use arbitrary direction
                    dh_dx = 1;
                    dh_dy = 0;
                end
                
            elseif strcmp(obstacle.type, 'rectangle')
                % Distance to closest point on rectangle
                % Clamp point to rectangle
                x_clamped = max(obstacle.x_min, min(x, obstacle.x_max));
                y_clamped = max(obstacle.y_min, min(y, obstacle.y_max));
                
                dx = x - x_clamped;
                dy = y - y_clamped;
                dist = sqrt(dx^2 + dy^2);
                
                % Barrier function
                h = dist - obj.safety_margin;
                
                % Gradient
                if dist > 1e-6
                    dh_dx = dx / dist;
                    dh_dy = dy / dist;
                else
                    % Inside rectangle, point outward
                    dh_dx = 1;
                    dh_dy = 0;
                end
            else
                % Unknown obstacle type
                h = 1.0;
                dh_dx = 0;
                dh_dy = 0;
            end
        end
        
        function is_safe = check_safety(obj, state)
            % Check if current state is safe (no collisions)
            x = state(1);
            y = state(2);
            
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                h = obj.compute_barrier(x, y, obs);
                
                if h < 0
                    is_safe = false;
                    return;
                end
            end
            
            is_safe = true;
        end
        
        function min_dist = get_min_obstacle_distance(obj, state)
            % Get minimum distance to any obstacle
            % Note: h already includes safety margin, so we return h directly
            x = state(1);
            y = state(2);
            
            min_dist = inf;
            
            for i = 1:length(obj.env.obstacles)
                obs = obj.env.obstacles{i};
                h = obj.compute_barrier(x, y, obs);
                % h already includes safety_margin in its computation
                % h = dist - (radius + safety_margin)
                % So actual clearance = h (not h + safety_margin)
                min_dist = min(min_dist, h);
            end
        end
    end
end

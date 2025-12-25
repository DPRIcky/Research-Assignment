classdef HybridAStarPlanner
    % HybridAStarPlanner - Hybrid A* path planning for vehicles
    % Considers vehicle kinematics and orientation
    
    properties
        env                 % Environment object
        resolution          % Grid resolution (m/cell)
        angle_resolution    % Heading angle resolution (rad)
        vehicle_length      % Vehicle length for collision checking
        turning_radius      % Minimum turning radius
        num_headings        % Number of discrete headings
        heuristic_weight    % Heuristic weight (epsilon for weighted A*)
    end
    
    methods
        function obj = HybridAStarPlanner(env, resolution, vehicle_length)
            % Constructor
            if nargin < 2
                resolution = 1.0;
            end
            if nargin < 3
                vehicle_length = 2.5;
            end
            
            obj.env = env;
            obj.resolution = resolution;
            obj.vehicle_length = vehicle_length;
            obj.turning_radius = 5.0;  % meters
            obj.num_headings = 72;      % 5-degree resolution
            obj.angle_resolution = 2*pi / obj.num_headings;
            obj.heuristic_weight = 2.0;  % Weighted A* for faster search
        end
        
        function [path, stats] = plan(obj, start, goal)
            % Plan path from start to goal using Hybrid A*
            % start/goal: [x, y, theta] (theta in radians)
            
            tic;
            
            % Ensure start and goal have heading
            if length(start) == 2
                start = [start(:)', 0];
            end
            if length(goal) == 2
                % Calculate heading towards goal from start
                goal = [goal(:)', atan2(goal(2)-start(2), goal(1)-start(1))];
            end
            
            % Initialize open and closed sets
            open_set = containers.Map('KeyType', 'char', 'ValueType', 'any');
            closed_set = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            % Create start node
            start_key = obj.state_to_key(start);
            start_node = struct('state', start, ...
                               'g', 0, ...
                               'h', obj.compute_heuristic(start, goal), ...
                               'f', 0, ...
                               'parent', []);
            start_node.f = start_node.g + obj.heuristic_weight * start_node.h;
            open_set(start_key) = start_node;
            
            nodes_expanded = 0;
            
            % Hybrid A* main loop
            while open_set.Count > 0
                % Find node with lowest f-score
                current_key = obj.find_lowest_f(open_set);
                current_node = open_set(current_key);
                
                % Check if goal reached
                if obj.is_goal_reached(current_node.state, goal)
                    path = obj.reconstruct_path(current_node);
                    stats = struct('nodes_expanded', nodes_expanded, ...
                                  'path_length', obj.compute_path_length(path), ...
                                  'runtime', toc, ...
                                  'success', true);
                    return;
                end
                
                % Move current from open to closed
                remove(open_set, current_key);
                closed_set(current_key) = current_node;
                nodes_expanded = nodes_expanded + 1;
                
                % Expand with vehicle motion primitives
                successors = obj.get_successors(current_node.state);
                
                for i = 1:size(successors, 1)
                    successor_state = successors(i, :);
                    successor_key = obj.state_to_key(successor_state);
                    
                    % Skip if in closed set
                    if isKey(closed_set, successor_key)
                        continue;
                    end
                    
                    % Check collision
                    if obj.check_state_collision(successor_state)
                        continue;
                    end
                    
                    % Calculate cost
                    move_cost = norm(successor_state(1:2) - current_node.state(1:2));
                    tentative_g = current_node.g + move_cost;
                    
                    % Check if this path is better
                    if ~isKey(open_set, successor_key)
                        % New node
                        successor_node = struct('state', successor_state, ...
                                               'g', tentative_g, ...
                                               'h', obj.compute_heuristic(successor_state, goal), ...
                                               'f', 0, ...
                                               'parent', current_node);
                        successor_node.f = successor_node.g + obj.heuristic_weight * successor_node.h;
                        open_set(successor_key) = successor_node;
                    else
                        % Update if better
                        successor_node = open_set(successor_key);
                        if tentative_g < successor_node.g
                            successor_node.g = tentative_g;
                            successor_node.f = successor_node.g + obj.heuristic_weight * successor_node.h;
                            successor_node.parent = current_node;
                            open_set(successor_key) = successor_node;
                        end
                    end
                end
                
                % Terminate if taking too long
                if nodes_expanded > 10000
                    break;
                end
            end
            
            % No path found
            path = [];
            stats = struct('nodes_expanded', nodes_expanded, ...
                          'path_length', inf, ...
                          'runtime', toc, ...
                          'success', false);
        end
        
        function successors = get_successors(obj, state)
            % Generate successor states using motion primitives
            % Forward, forward-left, forward-right
            
            x = state(1);
            y = state(2);
            theta = state(3);
            
            step_size = obj.resolution * 2;  % 2 grid cells
            
            % Motion primitives: straight, left turn, right turn
            steering_angles = [0, pi/6, -pi/6];  % 0, 30, -30 degrees
            
            successors = [];
            for steer = steering_angles
                % Simple bicycle model for next state
                if abs(steer) < 1e-6
                    % Straight motion
                    x_new = x + step_size * cos(theta);
                    y_new = y + step_size * sin(theta);
                    theta_new = theta;
                else
                    % Curved motion
                    turn_radius = obj.turning_radius;
                    arc_length = step_size;
                    d_theta = arc_length / turn_radius * sign(steer);
                    
                    x_new = x + turn_radius * (sin(theta + d_theta) - sin(theta)) * sign(steer);
                    y_new = y - turn_radius * (cos(theta + d_theta) - cos(theta)) * sign(steer);
                    theta_new = theta + d_theta;
                end
                
                % Normalize angle
                theta_new = atan2(sin(theta_new), cos(theta_new));
                
                successors = [successors; x_new, y_new, theta_new];
            end
        end
        
        function h = compute_heuristic(obj, state, goal)
            % Non-holonomic heuristic (Euclidean distance)
            h = norm(state(1:2) - goal(1:2));
        end
        
        function collision = check_state_collision(obj, state)
            % Check collision for a state including vehicle footprint
            x = state(1);
            y = state(2);
            theta = state(3);
            
            % Check vehicle center
            if obj.env.check_collision(x, y, obj.vehicle_length/2)
                collision = true;
                return;
            end
            
            % Check vehicle corners
            half_length = obj.vehicle_length / 2;
            half_width = obj.vehicle_length / 3;
            
            corners = [half_length, half_width;
                      half_length, -half_width;
                      -half_length, half_width;
                      -half_length, -half_width];
            
            for i = 1:size(corners, 1)
                % Rotate corner by vehicle heading
                rot_corner = [cos(theta) -sin(theta); sin(theta) cos(theta)] * corners(i,:)';
                corner_x = x + rot_corner(1);
                corner_y = y + rot_corner(2);
                
                if obj.env.check_collision(corner_x, corner_y, 0.5)
                    collision = true;
                    return;
                end
            end
            
            collision = false;
        end
        
        function reached = is_goal_reached(obj, state, goal)
            % Check if goal is reached (position and heading)
            pos_dist = norm(state(1:2) - goal(1:2));
            angle_diff = abs(atan2(sin(state(3) - goal(3)), cos(state(3) - goal(3))));
            
            reached = (pos_dist < obj.resolution * 2) && (angle_diff < pi/6);
        end
        
        function key = state_to_key(obj, state)
            % Convert continuous state to discrete key
            x_idx = round(state(1) / obj.resolution);
            y_idx = round(state(2) / obj.resolution);
            theta_idx = round(state(3) / obj.angle_resolution);
            
            key = sprintf('%d_%d_%d', x_idx, y_idx, theta_idx);
        end
        
        function lowest_key = find_lowest_f(~, open_set)
            % Find key with lowest f-score
            keys = open_set.keys;
            lowest_f = inf;
            lowest_key = '';
            
            for i = 1:length(keys)
                node = open_set(keys{i});
                if node.f < lowest_f
                    lowest_f = node.f;
                    lowest_key = keys{i};
                end
            end
        end
        
        function path = reconstruct_path(obj, goal_node)
            % Reconstruct path from goal to start
            path = [];
            current = goal_node;
            
            while ~isempty(current)
                path = [current.state; path];
                if isempty(current.parent)
                    break;
                end
                current = current.parent;
            end
        end
        
        function length = compute_path_length(~, path)
            % Compute total path length
            length = 0;
            for i = 1:size(path, 1)-1
                length = length + norm(path(i+1, 1:2) - path(i, 1:2));
            end
        end
    end
end

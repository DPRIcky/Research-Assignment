classdef RRTStarPlanner
    % RRTStarPlanner - RRT* path planning algorithm
    % Sampling-based planner with rewiring for optimality
    
    properties
        env                 % Environment object
        max_iterations      % Maximum number of iterations
        step_size          % Step size for extending tree
        search_radius      % Radius for rewiring neighbors
        goal_sample_rate   % Probability of sampling goal
        goal_threshold     % Distance threshold to reach goal
    end
    
    properties (Access = private)
        nodes              % Tree nodes (positions)
        parents            % Parent indices for each node
        costs              % Cost to reach each node from start
    end
    
    methods
        function obj = RRTStarPlanner(env, resolution_or_max_iter, step_size)
            % Constructor
            % Can be called as:
            %   RRTStarPlanner(env) - use defaults
            %   RRTStarPlanner(env, max_iter) - specify max iterations
            %   RRTStarPlanner(env, max_iter, step_size) - specify both
            
            if nargin < 2
                max_iter = 3000;
                step_size = 2.0;
            elseif nargin < 3
                % If second arg is small (< 10), treat as resolution
                % Otherwise treat as max_iter
                if resolution_or_max_iter < 10
                    % Likely a resolution parameter (e.g., 1.0)
                    max_iter = 3000;
                    step_size = 2.0;
                else
                    max_iter = resolution_or_max_iter;
                    step_size = 2.0;
                end
            else
                max_iter = resolution_or_max_iter;
                step_size = step_size;
            end
            
            obj.env = env;
            obj.max_iterations = max_iter;
            obj.step_size = step_size;
            obj.search_radius = 5.0;   % Rewiring radius
            obj.goal_sample_rate = 0.1; % 10% chance to sample goal
            obj.goal_threshold = 1.5;   % Goal reached threshold
        end
        
        function [path, stats] = plan(obj, start, goal)
            % Plan path from start to goal using RRT*
            % Returns path as Nx2 matrix [x, y] and statistics
            
            tic;
            
            % Initialize tree with start node
            obj.nodes = start(:)';
            obj.parents = 0;
            obj.costs = 0;
            
            goal_node_idx = -1;
            best_cost = inf;
            iterations = 0;
            
            % RRT* main loop
            for iter = 1:obj.max_iterations
                iterations = iter;
                
                % Sample random point (with goal bias)
                if rand() < obj.goal_sample_rate
                    x_rand = goal(:)';
                else
                    x_rand = obj.sample_random_point();
                end
                
                % Find nearest node
                nearest_idx = obj.find_nearest(x_rand);
                x_nearest = obj.nodes(nearest_idx, :);
                
                % Steer towards random point
                x_new = obj.steer(x_nearest, x_rand);
                
                % Check collision
                if obj.is_collision_free(x_nearest, x_new)
                    % Find nearby nodes for rewiring
                    near_indices = obj.find_near(x_new);
                    
                    % Choose best parent
                    [min_cost, min_idx] = obj.choose_parent(x_new, nearest_idx, near_indices);
                    
                    % Add new node
                    new_idx = size(obj.nodes, 1) + 1;
                    obj.nodes(new_idx, :) = x_new;
                    obj.parents(new_idx) = min_idx;
                    obj.costs(new_idx) = min_cost;
                    
                    % Rewire tree
                    obj = obj.rewire(new_idx, near_indices);
                    
                    % Check if goal reached
                    if norm(x_new - goal(:)') < obj.goal_threshold
                        current_cost = obj.costs(new_idx) + norm(x_new - goal(:)');
                        if current_cost < best_cost
                            goal_node_idx = new_idx;
                            best_cost = current_cost;
                        end
                    end
                end
                
                % Early termination if good path found
                if goal_node_idx > 0 && iter > 1000
                    break;
                end
            end
            
            % Extract path
            if goal_node_idx > 0
                path = obj.extract_path(goal_node_idx, goal);
                stats = struct('iterations', iterations, ...
                              'nodes_created', size(obj.nodes, 1), ...
                              'path_length', obj.compute_path_length(path), ...
                              'runtime', toc, ...
                              'success', true);
            else
                path = [];
                stats = struct('iterations', iterations, ...
                              'nodes_created', size(obj.nodes, 1), ...
                              'path_length', inf, ...
                              'runtime', toc, ...
                              'success', false);
            end
        end
        
        function x_rand = sample_random_point(obj)
            % Sample random point in environment
            x = rand() * obj.env.map_size(1);
            y = rand() * obj.env.map_size(2);
            x_rand = [x, y];
        end
        
        function nearest_idx = find_nearest(obj, x)
            % Find nearest node in tree to point x
            distances = vecnorm(obj.nodes - x, 2, 2);
            [~, nearest_idx] = min(distances);
        end
        
        function x_new = steer(obj, x_from, x_to)
            % Steer from x_from towards x_to with step_size limit
            direction = x_to - x_from;
            distance = norm(direction);
            
            if distance < obj.step_size
                x_new = x_to;
            else
                x_new = x_from + (direction / distance) * obj.step_size;
            end
        end
        
        function collision_free = is_collision_free(obj, x_from, x_to)
            % Check if path from x_from to x_to is collision free
            % Sample points along the path
            num_checks = ceil(norm(x_to - x_from) / 0.5);
            
            for i = 0:num_checks
                t = i / max(num_checks, 1);
                x_check = x_from + t * (x_to - x_from);
                
                if obj.env.check_collision(x_check(1), x_check(2))
                    collision_free = false;
                    return;
                end
            end
            collision_free = true;
        end
        
        function near_indices = find_near(obj, x)
            % Find all nodes within search_radius of x
            distances = vecnorm(obj.nodes - x, 2, 2);
            near_indices = find(distances < obj.search_radius);
        end
        
        function [min_cost, min_idx] = choose_parent(obj, x_new, nearest_idx, near_indices)
            % Choose best parent from nearby nodes
            min_cost = obj.costs(nearest_idx) + norm(obj.nodes(nearest_idx,:) - x_new);
            min_idx = nearest_idx;
            
            for i = 1:length(near_indices)
                idx = near_indices(i);
                potential_cost = obj.costs(idx) + norm(obj.nodes(idx,:) - x_new);
                
                if potential_cost < min_cost
                    if obj.is_collision_free(obj.nodes(idx,:), x_new)
                        min_cost = potential_cost;
                        min_idx = idx;
                    end
                end
            end
        end
        
        function obj = rewire(obj, new_idx, near_indices)
            % Rewire tree through new node if it provides better path
            x_new = obj.nodes(new_idx, :);
            
            for i = 1:length(near_indices)
                idx = near_indices(i);
                if idx == new_idx
                    continue;
                end
                
                x_near = obj.nodes(idx, :);
                potential_cost = obj.costs(new_idx) + norm(x_new - x_near);
                
                if potential_cost < obj.costs(idx)
                    if obj.is_collision_free(x_new, x_near)
                        obj.parents(idx) = new_idx;
                        obj.costs(idx) = potential_cost;
                    end
                end
            end
        end
        
        function path = extract_path(obj, goal_idx, goal)
            % Extract path from start to goal
            path = goal(:)';
            current_idx = goal_idx;
            
            while current_idx > 0
                path = [obj.nodes(current_idx, :); path];
                current_idx = obj.parents(current_idx);
            end
        end
        
        function length = compute_path_length(~, path)
            % Compute total path length
            length = 0;
            for i = 1:size(path, 1)-1
                length = length + norm(path(i+1, :) - path(i, :));
            end
        end
        
        function plot_tree(obj)
            % Visualize the RRT* tree
            if size(obj.nodes, 1) < 2
                return;
            end
            
            hold on;
            for i = 2:size(obj.nodes, 1)
                parent_idx = obj.parents(i);
                if parent_idx > 0 && parent_idx <= size(obj.nodes, 1)
                    plot([obj.nodes(i,1), obj.nodes(parent_idx,1)], ...
                         [obj.nodes(i,2), obj.nodes(parent_idx,2)], ...
                         'c-', 'LineWidth', 0.5);
                end
            end
            if size(obj.nodes, 1) > 0
                plot(obj.nodes(:,1), obj.nodes(:,2), 'c.', 'MarkerSize', 3);
            end
            hold off;
        end
    end
end

classdef AStarPlanner
    % AStarPlanner - A* path planning algorithm
    % Grid-based pathfinding with configurable heuristics
    
    properties
        env             % Environment object
        resolution      % Grid resolution (m/cell)
        heuristic       % 'euclidean', 'manhattan', 'diagonal'
        allow_diagonal  % Allow diagonal movements
    end
    
    methods
        function obj = AStarPlanner(env, resolution, heuristic)
            % Constructor
            if nargin < 2
                resolution = 1.0;
            end
            if nargin < 3
                heuristic = 'euclidean';
            end
            
            obj.env = env;
            obj.resolution = resolution;
            obj.heuristic = heuristic;
            obj.allow_diagonal = true;
        end
        
        function [path, stats] = plan(obj, start, goal)
            % Plan path from start to goal using A*
            % Returns path as Nx2 matrix [x, y] and statistics
            
            tic;
            
            % Convert to grid coordinates
            start_grid = obj.world_to_grid(start);
            goal_grid = obj.world_to_grid(goal);
            
            % Check if start/goal are valid
            if obj.env.check_collision(start(1), start(2))
                error('Start position is in collision');
            end
            if obj.env.check_collision(goal(1), goal(2))
                error('Goal position is in collision');
            end
            
            % Initialize open and closed sets
            open_set = containers.Map('KeyType', 'char', 'ValueType', 'any');
            closed_set = containers.Map('KeyType', 'char', 'ValueType', 'any');
            
            % Create start node
            start_key = obj.grid_to_key(start_grid);
            start_node = struct('pos', start_grid, ...
                               'g', 0, ...
                               'h', obj.compute_heuristic(start_grid, goal_grid), ...
                               'f', 0, ...
                               'parent', []);
            start_node.f = start_node.g + start_node.h;
            open_set(start_key) = start_node;
            
            nodes_expanded = 0;
            
            % A* main loop
            while open_set.Count > 0
                % Find node with lowest f-score
                current_key = obj.find_lowest_f(open_set);
                current_node = open_set(current_key);
                
                % Check if goal reached
                if isequal(current_node.pos, goal_grid)
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
                
                % Expand neighbors
                neighbors = obj.get_neighbors(current_node.pos);
                for i = 1:size(neighbors, 1)
                    neighbor_pos = neighbors(i, :);
                    neighbor_key = obj.grid_to_key(neighbor_pos);
                    
                    % Skip if in closed set
                    if isKey(closed_set, neighbor_key)
                        continue;
                    end
                    
                    % Check collision
                    world_pos = obj.grid_to_world(neighbor_pos);
                    if obj.env.check_collision(world_pos(1), world_pos(2))
                        continue;
                    end
                    
                    % Calculate tentative g-score
                    move_cost = norm(neighbor_pos - current_node.pos) * obj.resolution;
                    tentative_g = current_node.g + move_cost;
                    
                    % Check if this path to neighbor is better
                    if ~isKey(open_set, neighbor_key)
                        % New node
                        neighbor_node = struct('pos', neighbor_pos, ...
                                             'g', tentative_g, ...
                                             'h', obj.compute_heuristic(neighbor_pos, goal_grid), ...
                                             'f', 0, ...
                                             'parent', current_node);
                        neighbor_node.f = neighbor_node.g + neighbor_node.h;
                        open_set(neighbor_key) = neighbor_node;
                    else
                        % Update if better path found
                        neighbor_node = open_set(neighbor_key);
                        if tentative_g < neighbor_node.g
                            neighbor_node.g = tentative_g;
                            neighbor_node.f = neighbor_node.g + neighbor_node.h;
                            neighbor_node.parent = current_node;
                            open_set(neighbor_key) = neighbor_node;
                        end
                    end
                end
            end
            
            % No path found
            path = [];
            stats = struct('nodes_expanded', nodes_expanded, ...
                          'path_length', inf, ...
                          'runtime', toc, ...
                          'success', false);
        end
        
        function h = compute_heuristic(obj, pos1, pos2)
            % Compute heuristic distance between two grid positions
            dx = abs(pos1(1) - pos2(1));
            dy = abs(pos1(2) - pos2(2));
            
            switch obj.heuristic
                case 'euclidean'
                    h = sqrt(dx^2 + dy^2) * obj.resolution;
                case 'manhattan'
                    h = (dx + dy) * obj.resolution;
                case 'diagonal'
                    % Diagonal distance (Chebyshev)
                    h = max(dx, dy) * obj.resolution;
                otherwise
                    h = sqrt(dx^2 + dy^2) * obj.resolution;
            end
        end
        
        function neighbors = get_neighbors(obj, pos)
            % Get valid neighboring grid cells
            if obj.allow_diagonal
                % 8-connected grid
                offsets = [-1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1];
            else
                % 4-connected grid
                offsets = [-1 0; 0 -1; 0 1; 1 0];
            end
            
            neighbors = pos + offsets;
        end
        
        function key = grid_to_key(~, pos)
            % Convert grid position to unique string key
            key = sprintf('%d_%d', pos(1), pos(2));
        end
        
        function lowest_key = find_lowest_f(~, open_set)
            % Find key with lowest f-score in open set
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
            path_grid = [];
            current = goal_node;
            
            while ~isempty(current)
                path_grid = [current.pos; path_grid];
                if isempty(current.parent)
                    break;
                end
                current = current.parent;
            end
            
            % Convert to world coordinates
            path = zeros(size(path_grid));
            for i = 1:size(path_grid, 1)
                world_pos = obj.grid_to_world(path_grid(i, :));
                path(i, :) = world_pos;
            end
        end
        
        function length = compute_path_length(~, path)
            % Compute total path length
            length = 0;
            for i = 1:size(path, 1)-1
                length = length + norm(path(i+1, :) - path(i, :));
            end
        end
        
        function grid_pos = world_to_grid(obj, world_pos)
            % Convert world coordinates to grid coordinates
            grid_pos = round(world_pos / obj.resolution);
        end
        
        function world_pos = grid_to_world(obj, grid_pos)
            % Convert grid coordinates to world coordinates
            world_pos = grid_pos * obj.resolution;
        end
    end
end

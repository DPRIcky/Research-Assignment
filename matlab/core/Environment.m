classdef Environment
    % Environment - Manages the simulation environment (map, obstacles)
    
    properties
        map_size        % [width, height] in meters
        obstacles       % Cell array of obstacles [x, y, radius] or [x, y, width, height]
        start_pos       % [x, y] starting position
        goal_pos        % [x, y] goal position
        resolution      % Grid resolution for planning (m/cell)
    end
    
    methods
        function obj = Environment(map_size, resolution)
            % Constructor
            if nargin < 1
                map_size = [100, 100];  % Default 100x100 m
            end
            if nargin < 2
                resolution = 1.0;  % 1 meter per cell
            end
            
            obj.map_size = map_size;
            obj.resolution = resolution;
            obj.obstacles = {};
            obj.start_pos = [10, 10];
            obj.goal_pos = [90, 90];
        end
        
        function obj = add_circular_obstacle(obj, x, y, radius)
            % Add circular obstacle
            obj.obstacles{end+1} = struct('type', 'circle', ...
                                          'x', x, 'y', y, 'radius', radius);
        end
        
        function obj = add_rectangular_obstacle(obj, x, y, width, height)
            % Add rectangular obstacle (x, y is bottom-left corner)
            obj.obstacles{end+1} = struct('type', 'rectangle', ...
                                          'x', x, 'y', y, ...
                                          'width', width, 'height', height);
        end
        
        function obj = set_start_goal(obj, start, goal)
            % Set start and goal positions
            obj.start_pos = start;
            obj.goal_pos = goal;
        end
        
        function collision = check_collision(obj, x, y, safety_margin)
            % Check if point (x,y) collides with obstacles
            if nargin < 4
                safety_margin = 0.5;  % default 0.5m safety margin
            end
            
            collision = false;
            
            % Check bounds
            if x < 0 || x > obj.map_size(1) || y < 0 || y > obj.map_size(2)
                collision = true;
                return;
            end
            
            % Check each obstacle
            for i = 1:length(obj.obstacles)
                obs = obj.obstacles{i};
                
                if strcmp(obs.type, 'circle')
                    dist = sqrt((x - obs.x)^2 + (y - obs.y)^2);
                    if dist < (obs.radius + safety_margin)
                        collision = true;
                        return;
                    end
                    
                elseif strcmp(obs.type, 'rectangle')
                    % Check if point is inside expanded rectangle
                    if x >= (obs.x - safety_margin) && ...
                       x <= (obs.x + obs.width + safety_margin) && ...
                       y >= (obs.y - safety_margin) && ...
                       y <= (obs.y + obs.height + safety_margin)
                        collision = true;
                        return;
                    end
                end
            end
        end
        
        function collision = check_path_collision(obj, path, safety_margin)
            % Check if a path collides with obstacles
            % path: Nx2 matrix of [x, y] points
            if nargin < 3
                safety_margin = 0.5;
            end
            
            collision = false;
            for i = 1:size(path, 1)
                if obj.check_collision(path(i,1), path(i,2), safety_margin)
                    collision = true;
                    return;
                end
            end
        end
        
        function plot(obj, show_grid)
            % Visualize the environment
            if nargin < 2
                show_grid = false;
            end
            
            hold on;
            axis equal;
            xlim([0 obj.map_size(1)]);
            ylim([0 obj.map_size(2)]);
            xlabel('X (m)');
            ylabel('Y (m)');
            title('Environment');
            grid on;
            
            % Plot obstacles
            for i = 1:length(obj.obstacles)
                obs = obj.obstacles{i};
                
                if strcmp(obs.type, 'circle')
                    theta = linspace(0, 2*pi, 50);
                    x_circle = obs.x + obs.radius * cos(theta);
                    y_circle = obs.y + obs.radius * sin(theta);
                    fill(x_circle, y_circle, [0.3 0.3 0.3], 'FaceAlpha', 0.7, ...
                         'DisplayName', sprintf('Obstacle %d', i));
                    
                elseif strcmp(obs.type, 'rectangle')
                    % Use fill for rectangles to support DisplayName
                    x_rect = [obs.x, obs.x+obs.width, obs.x+obs.width, obs.x, obs.x];
                    y_rect = [obs.y, obs.y, obs.y+obs.height, obs.y+obs.height, obs.y];
                    fill(x_rect, y_rect, [0.3 0.3 0.3], 'FaceAlpha', 0.7, ...
                         'DisplayName', sprintf('Obstacle %d', i));
                end
            end
            
            % Plot start and goal
            plot(obj.start_pos(1), obj.start_pos(2), 'go', ...
                 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(obj.goal_pos(1), obj.goal_pos(2), 'r*', ...
                 'MarkerSize', 15, 'LineWidth', 2, 'DisplayName', 'Goal');
            
            legend('Location', 'best');
            hold off;
        end
        
        function env_simple = create_simple_scenario(obj)
            % Create a simple test scenario with a few obstacles
            obj = obj.add_circular_obstacle(30, 30, 8);
            obj = obj.add_circular_obstacle(50, 50, 10);
            obj = obj.add_circular_obstacle(70, 40, 6);
            obj = obj.add_rectangular_obstacle(40, 70, 20, 10);
            obj = obj.set_start_goal([10, 10], [90, 90]);
            env_simple = obj;
        end
    end
end

classdef VehicleDynamics
    % VehicleDynamics - Simple 2D vehicle dynamics model
    % Supports both kinematic bicycle model and differential drive
    
    properties
        type            % 'bicycle' or 'differential'
        L               % Wheelbase (for bicycle model)
        max_speed       % Maximum velocity
        max_accel       % Maximum acceleration
        max_steer       % Maximum steering angle (for bicycle model)
        max_omega       % Maximum angular velocity (for differential drive)
        dt              % Time step
    end
    
    methods
        function obj = VehicleDynamics(type, dt)
            % Constructor
            if nargin < 1
                type = 'bicycle';
            end
            if nargin < 2
                dt = 0.1;
            end
            
            obj.type = type;
            obj.dt = dt;
            
            % Default parameters
            if strcmp(type, 'bicycle')
                obj.L = 2.5;            % m
                obj.max_speed = 5.0;    % m/s
                obj.max_accel = 2.0;    % m/s^2
                obj.max_steer = pi/4;   % rad (45 degrees)
            else % differential drive
                obj.max_speed = 2.0;    % m/s
                obj.max_accel = 1.0;    % m/s^2
                obj.max_omega = pi/2;   % rad/s
            end
        end
        
        function state_next = step(obj, state, control)
            % Simulate one time step
            % state: [x, y, theta, v] for bicycle
            %        [x, y, theta] for differential
            % control: [a, delta] for bicycle (accel, steering)
            %          [v, omega] for differential (linear vel, angular vel)
            
            if strcmp(obj.type, 'bicycle')
                state_next = obj.bicycle_model(state, control);
            else
                state_next = obj.differential_model(state, control);
            end
        end
        
        function state_next = bicycle_model(obj, state, control)
            % Kinematic bicycle model
            x = state(1);
            y = state(2);
            theta = state(3);
            v = state(4);
            
            a = control(1);      % acceleration
            delta = control(2);  % steering angle
            
            % Limit inputs
            a = max(min(a, obj.max_accel), -obj.max_accel);
            delta = max(min(delta, obj.max_steer), -obj.max_steer);
            
            % Update velocity
            v_next = v + a * obj.dt;
            v_next = max(min(v_next, obj.max_speed), -obj.max_speed);
            
            % Update state using bicycle kinematics
            x_next = x + v * cos(theta) * obj.dt;
            y_next = y + v * sin(theta) * obj.dt;
            theta_next = theta + (v / obj.L) * tan(delta) * obj.dt;
            
            % Normalize angle
            theta_next = atan2(sin(theta_next), cos(theta_next));
            
            state_next = [x_next; y_next; theta_next; v_next];
        end
        
        function state_next = differential_model(obj, state, control)
            % Differential drive model
            x = state(1);
            y = state(2);
            theta = state(3);
            
            v = control(1);      % linear velocity
            omega = control(2);  % angular velocity
            
            % Limit inputs
            v = max(min(v, obj.max_speed), -obj.max_speed);
            omega = max(min(omega, obj.max_omega), -obj.max_omega);
            
            % Update state
            x_next = x + v * cos(theta) * obj.dt;
            y_next = y + v * sin(theta) * obj.dt;
            theta_next = theta + omega * obj.dt;
            
            % Normalize angle
            theta_next = atan2(sin(theta_next), cos(theta_next));
            
            state_next = [x_next; y_next; theta_next];
        end
    end
end

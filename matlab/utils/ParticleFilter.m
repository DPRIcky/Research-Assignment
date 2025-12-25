classdef ParticleFilter
    % ParticleFilter - Particle Filter for nonlinear state estimation
    % Uses Sequential Importance Resampling (SIR) algorithm
    
    properties
        dt                % Time step
        particles         % Particle states (4 x N_particles)
        weights           % Particle weights (N_particles x 1)
        N                 % Number of particles
        Q                 % Process noise covariance
        R                 % Measurement noise covariance
        L                 % Vehicle wheelbase
        resample_threshold  % Effective sample size threshold for resampling
    end
    
    methods
        function obj = ParticleFilter(dt, process_noise, measurement_noise, N_particles, L)
            % Constructor
            % dt: time step
            % process_noise: 4x4 process noise covariance matrix
            % measurement_noise: 3x3 measurement noise covariance matrix
            % N_particles: number of particles (default: 500)
            % L: vehicle wheelbase (default: 2.5)
            
            obj.dt = dt;
            obj.Q = process_noise;
            obj.R = measurement_noise;
            
            if nargin < 4 || isempty(N_particles)
                obj.N = 500;
            else
                obj.N = N_particles;
            end
            
            if nargin < 5
                obj.L = 2.5;
            else
                obj.L = L;
            end
            
            % Initialize particles and weights
            obj.particles = zeros(4, obj.N);
            obj.weights = ones(obj.N, 1) / obj.N;
            
            % Resampling threshold (Neff < N/2)
            obj.resample_threshold = obj.N / 2;
        end
        
        function obj = initialize(obj, initial_state, initial_spread)
            % Initialize particles around initial state
            % initial_spread: standard deviation for initialization (default: 0.5)
            
            if nargin < 3
                initial_spread = 0.5;
            end
            
            % Initialize particles with Gaussian noise around initial state
            for i = 1:obj.N
                obj.particles(1, i) = initial_state(1) + randn() * initial_spread;
                obj.particles(2, i) = initial_state(2) + randn() * initial_spread;
                obj.particles(3, i) = initial_state(3) + randn() * initial_spread * 0.2;
                obj.particles(4, i) = initial_state(4) + randn() * initial_spread * 0.1;
            end
            
            % Uniform weights
            obj.weights = ones(obj.N, 1) / obj.N;
        end
        
        function [x_new] = process_model(obj, x, u, add_noise)
            % Nonlinear process model: bicycle kinematics
            % x: state [x, y, theta, v]
            % u: control [v_cmd, omega_cmd]
            % add_noise: whether to add process noise
            
            if nargin < 4
                add_noise = true;
            end
            
            v = u(1);
            omega = u(2);
            
            % Bicycle model
            x_new = zeros(4, 1);
            x_new(1) = x(1) + v * cos(x(3)) * obj.dt;
            x_new(2) = x(2) + v * sin(x(3)) * obj.dt;
            x_new(3) = x(3) + omega * obj.dt;
            x_new(4) = v;
            
            % Add process noise
            if add_noise
                noise = mvnrnd(zeros(4, 1), obj.Q)';
                x_new = x_new + noise;
            end
            
            % Normalize angle to [-pi, pi]
            x_new(3) = atan2(sin(x_new(3)), cos(x_new(3)));
        end
        
        function [likelihood] = measurement_likelihood(obj, x, z_meas)
            % Compute likelihood of measurement given state
            % x: particle state
            % z_meas: measurement [x, y, theta]
            
            % Predicted measurement
            z_pred = x(1:3);
            
            % Innovation
            innov = z_meas - z_pred;
            innov(3) = atan2(sin(innov(3)), cos(innov(3)));
            
            % Multivariate Gaussian likelihood
            likelihood = mvnpdf(innov', zeros(1, 3), obj.R);
        end
        
        function obj = predict(obj, u)
            % Prediction step: propagate particles through process model
            
            for i = 1:obj.N
                obj.particles(:, i) = obj.process_model(obj.particles(:, i), u, true);
            end
        end
        
        function obj = update(obj, z_meas)
            % Update step: weight particles based on measurement likelihood
            
            for i = 1:obj.N
                obj.weights(i) = obj.weights(i) * ...
                    obj.measurement_likelihood(obj.particles(:, i), z_meas);
            end
            
            % Normalize weights
            weight_sum = sum(obj.weights);
            if weight_sum > 1e-10
                obj.weights = obj.weights / weight_sum;
            else
                % Reinitialize if all weights are too small
                obj.weights = ones(obj.N, 1) / obj.N;
            end
            
            % Check effective sample size and resample if needed
            Neff = 1 / sum(obj.weights.^2);
            if Neff < obj.resample_threshold
                obj = obj.resample();
            end
        end
        
        function obj = resample(obj)
            % Systematic resampling
            
            % Cumulative sum of weights
            cumsum_weights = cumsum(obj.weights);
            
            % Generate systematic samples
            new_particles = zeros(4, obj.N);
            u0 = rand() / obj.N;
            
            i = 1;
            for j = 1:obj.N
                uj = u0 + (j - 1) / obj.N;
                while uj > cumsum_weights(i) && i < obj.N
                    i = i + 1;
                end
                new_particles(:, j) = obj.particles(:, i);
            end
            
            obj.particles = new_particles;
            obj.weights = ones(obj.N, 1) / obj.N;
        end
        
        function [state_estimate] = get_state_estimate(obj)
            % Get weighted mean state estimate
            
            state_estimate = zeros(4, 1);
            
            % Weighted mean for x, y, v
            state_estimate(1) = sum(obj.weights .* obj.particles(1, :)');
            state_estimate(2) = sum(obj.weights .* obj.particles(2, :)');
            state_estimate(4) = sum(obj.weights .* obj.particles(4, :)');
            
            % Circular mean for theta
            sin_sum = sum(obj.weights .* sin(obj.particles(3, :)'));
            cos_sum = sum(obj.weights .* cos(obj.particles(3, :)'));
            state_estimate(3) = atan2(sin_sum, cos_sum);
        end
        
        function [state_estimate, covariance] = get_state_with_covariance(obj)
            % Get state estimate with covariance
            
            state_estimate = obj.get_state_estimate();
            
            % Compute weighted covariance
            covariance = zeros(4, 4);
            for i = 1:obj.N
                diff = obj.particles(:, i) - state_estimate;
                diff(3) = atan2(sin(diff(3)), cos(diff(3)));
                covariance = covariance + obj.weights(i) * (diff * diff');
            end
        end
    end
end

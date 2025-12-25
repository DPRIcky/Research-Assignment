classdef StateEstimator
    % StateEstimator - Provides state information (ground truth, noisy, or from logs)
    % This class simulates different state estimation scenarios
    
    properties
        mode            % 'perfect', 'noisy', 'external'
        noise_std       % Standard deviation for position and angle noise
        external_data   % Matrix of prerecorded states [t, x, y, theta, ...]
        current_idx     % Current index in external data
    end
    
    methods
        function obj = StateEstimator(mode, noise_std)
            % Constructor
            if nargin < 1
                mode = 'perfect';
            end
            if nargin < 2
                noise_std = [0.1, 0.1, 0.05];  % [x_std, y_std, theta_std]
            end
            
            obj.mode = mode;
            obj.noise_std = noise_std;
            obj.external_data = [];
            obj.current_idx = 1;
        end
        
        function obj = load_external_data(obj, filename)
            % Load external state data from file
            obj.external_data = load(filename);
            obj.current_idx = 1;
            obj.mode = 'external';
        end
        
        function estimated_state = get_state(obj, true_state, time)
            % Get state estimate based on mode
            % true_state: actual vehicle state
            % time: current simulation time (for external data lookup)
            
            switch obj.mode
                case 'perfect'
                    % Return perfect ground truth
                    estimated_state = true_state;
                    
                case 'noisy'
                    % Add Gaussian noise to ground truth
                    estimated_state = true_state;
                    estimated_state(1) = estimated_state(1) + randn() * obj.noise_std(1);
                    estimated_state(2) = estimated_state(2) + randn() * obj.noise_std(2);
                    estimated_state(3) = estimated_state(3) + randn() * obj.noise_std(3);
                    
                case 'external'
                    % Use prerecorded external data
                    if isempty(obj.external_data)
                        warning('No external data loaded. Using perfect state.');
                        estimated_state = true_state;
                    else
                        % Find closest time in external data
                        [~, idx] = min(abs(obj.external_data(:,1) - time));
                        estimated_state = obj.external_data(idx, 2:end)';
                    end
                    
                otherwise
                    error('Unknown state estimator mode: %s', obj.mode);
            end
        end
        
        function obj = set_noise_level(obj, noise_std)
            % Update noise standard deviation
            obj.noise_std = noise_std;
        end
    end
end

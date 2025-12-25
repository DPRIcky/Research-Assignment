function save_results(metrics, trajectory, controls, timestamps, config, filename)
    % save_results - Save simulation results to file
    %
    % Inputs:
    %   metrics: Metrics structure from compute_metrics()
    %   trajectory: Trajectory data (Nx4)
    %   controls: Control inputs (Nx2)
    %   timestamps: Time vector (Nx1)
    %   config: Configuration structure
    %   filename: Output filename (without extension)
    %
    % Outputs:
    %   Creates .mat and .csv files with results
    
    % Create results directory if it doesn't exist
    results_dir = 'results';
    if ~exist(results_dir, 'dir')
        mkdir(results_dir);
    end
    
    %% Save MATLAB format
    results = struct();
    results.metrics = metrics;
    results.trajectory = trajectory;
    results.controls = controls;
    results.timestamps = timestamps;
    results.config = config;
    results.timestamp = datetime('now');
    
    mat_file = fullfile(results_dir, [filename, '.mat']);
    save(mat_file, 'results');
    fprintf('✓ Saved MATLAB results to: %s\n', mat_file);
    
    %% Save metrics as CSV
    csv_file = fullfile(results_dir, [filename, '_metrics.csv']);
    
    % Convert metrics struct to table
    field_names = fieldnames(metrics);
    metric_names = cell(length(field_names), 1);
    metric_values = zeros(length(field_names), 1);
    
    for i = 1:length(field_names)
        metric_names{i} = field_names{i};
        value = metrics.(field_names{i});
        if islogical(value)
            metric_values(i) = double(value);
        else
            metric_values(i) = value;
        end
    end
    
    T = table(metric_names, metric_values, 'VariableNames', {'Metric', 'Value'});
    writetable(T, csv_file);
    fprintf('✓ Saved metrics CSV to: %s\n', csv_file);
    
    %% Save trajectory data as CSV
    traj_csv_file = fullfile(results_dir, [filename, '_trajectory.csv']);
    
    traj_table = table(timestamps, ...
                       trajectory(:,1), trajectory(:,2), trajectory(:,3), trajectory(:,4), ...
                       controls(:,1), controls(:,2), ...
                       'VariableNames', {'Time', 'X', 'Y', 'Theta', 'Velocity', ...
                                         'Control_V', 'Control_Omega'});
    writetable(traj_table, traj_csv_file);
    fprintf('✓ Saved trajectory CSV to: %s\n', traj_csv_file);
    
    %% Save configuration as text file
    config_file = fullfile(results_dir, [filename, '_config.txt']);
    fid = fopen(config_file, 'w');
    
    fprintf(fid, 'Simulation Configuration\n');
    fprintf(fid, '========================\n\n');
    
    config_fields = fieldnames(config);
    for i = 1:length(config_fields)
        field = config_fields{i};
        value = config.(field);
        
        if isnumeric(value)
            if length(value) == 1
                fprintf(fid, '%s: %.4f\n', field, value);
            else
                fprintf(fid, '%s: [%s]\n', field, num2str(value, '%.4f '));
            end
        elseif islogical(value)
            fprintf(fid, '%s: %s\n', field, iif(value, 'true', 'false'));
        elseif ischar(value)
            fprintf(fid, '%s: %s\n', field, value);
        end
    end
    
    fclose(fid);
    fprintf('✓ Saved configuration to: %s\n', config_file);
end

function result = iif(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end

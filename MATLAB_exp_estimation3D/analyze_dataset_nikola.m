% Analyze all CSV files in dataset_Nikola for 3D position estimation
% This script:
% 1. Iterates through all CSV files in Database/dataset_Nikola
% 2. Estimates direction vector for each position
% 3. Estimates distance using P_r_distance (stage='distance')
% 4. Calculates 3D position estimate
% 5. Compares estimated vs reference positions in a single figure

clear;
close all;
clc;

% ========== SPATIAL FILTERING CONFIGURATION ==========
% Set to true to enable spatial filtering based on X,Y ranges
spatial_filter_enabled = false;  % Set to true to enable filtering

% Define X range (L) and Y range (W) for spatial filtering
L = [-0.6, 0.6];    % X range [min, max] in meters
W = [-0.6, 0.6];    % Y range [min, max] in meters

% Note: When spatial_filter_enabled = true, only positions where:
%   L(1) <= x <= L(2) AND W(1) <= y <= W(2) will be processed
%   Z (altitude) is not considered in the filtering
% =====================================================

% Load calibration constant K
try
    load('calibration.mat', 'K_exp');
    K = K_exp;
    fprintf('Loaded calibration constant K = %.6f\n', K);
catch
    warning('Could not load calibration.mat. Using default K value.');
    K = 0.001; % Default value, adjust as needed
end

% Configuration
m = 3; % Lambertian order
transmitter_pos = [0, 0, 2]; % Fixed transmitter position

% Define which orientations to use (indices)
K_indices = [1,3,4,5,6]; % Orientation indices to use
% K_indices = [1,7,8,9,10]; % Orientation indices to use
% K_indices = [1,3,4,5,6,7,8,9,10]; % Orientation indices to use

% Get all CSV files in dataset_Nikola
dataset_dir = '../Database/dataset_Nikola/';
csv_files = dir(fullfile(dataset_dir, '*.csv'));
num_files = length(csv_files);

fprintf('Found %d CSV files in dataset_Nikola\n', num_files);

% Display spatial filtering status
if spatial_filter_enabled
    fprintf('\n=== SPATIAL FILTERING ENABLED ===\n');
    fprintf('X range: [%.2f, %.2f] m\n', L(1), L(2));
    fprintf('Y range: [%.2f, %.2f] m\n', W(1), W(2));
    fprintf('Only processing positions within these ranges.\n');
    fprintf('==================================\n\n');
else
    fprintf('Spatial filtering: DISABLED - Processing all files\n\n');
end

% Initialize arrays to store results
reference_positions = [];
estimated_positions = [];
successful_files = {};
failed_files = {};

% Process each CSV file
for file_idx = 1:num_files
    csv_file = fullfile(dataset_dir, csv_files(file_idx).name);
    
    % Extract receiver position from filename first (for filtering)
    [~, filename, ~] = fileparts(csv_files(file_idx).name);
    coords_str = strrep(filename, 'data_', '');
    coords = sscanf(coords_str, '%f_%f_%f');
    
    % Check if coordinates can be extracted
    if length(coords) ~= 3
        fprintf('Skipping file %d/%d: %s (cannot extract coordinates)\n', ...
                file_idx, num_files, csv_files(file_idx).name);
        failed_files{end+1} = csv_files(file_idx).name;
        continue;
    end
    
    receiver_x = coords(1);
    receiver_y = coords(2);
    receiver_z = coords(3);
    
    % Apply spatial filtering if enabled
    if spatial_filter_enabled
        % Check if position is within the specified X,Y ranges
        if receiver_x < L(1) || receiver_x > L(2) || ...
           receiver_y < W(1) || receiver_y > W(2)
            fprintf('Skipping file %d/%d: %s (outside spatial filter range)\n', ...
                    file_idx, num_files, csv_files(file_idx).name);
            fprintf('  Position: [%.2f, %.2f, %.2f] - Outside X:[%.2f,%.2f], Y:[%.2f,%.2f]\n', ...
                    receiver_x, receiver_y, receiver_z, L(1), L(2), W(1), W(2));
            continue;  % Skip this file
        end
    end
    
    fprintf('\n===========================================\n');
    fprintf('Processing file %d/%d: %s\n', file_idx, num_files, csv_files(file_idx).name);
    fprintf('===========================================\n');
    
    try
        receiver_pos = [receiver_x, receiver_y, receiver_z];
        fprintf('Reference position: [%.2f, %.2f, %.2f]\n', receiver_x, receiver_y, receiver_z);
        
        % Calculate real unit direction vector
        direction_vector = receiver_pos - transmitter_pos;
        real_distance = norm(direction_vector);
        direction_unit_vector = direction_vector / real_distance;
        
        % Read CSV data
        data = readtable(csv_file);
        
        % Process background data
        background_data = data(strcmp(data.stage, 'background'), :);
        if isempty(background_data)
            warning('No background data found, using 0');
            background_mean = 0;
        else
            background_mean = mean(background_data.medida_daq);
        end
        % Process distance stage data
        distance_data = data(strcmp(data.stage, 'distance'), :);
        if isempty(distance_data)
            warning('No distance data found for file %s', csv_files(file_idx).name);
            % Try to use direction data at (0,0) as fallback
            distance_data = data(strcmp(data.stage, 'direction') & ...
                                data.inclinacion == 0 & ...
                                data.azimuth == 0, :);
            if isempty(distance_data)
                error('No distance or direction(0,0) data available');
            end
        end
        
        % Calculate P_r_distance
        distance_mean = mean(distance_data.medida_daq);
        P_r_distance = abs(distance_mean - background_mean);
        
        if P_r_distance <= 0
            error('Invalid P_r_distance: %.6f', P_r_distance);
        end
        
        % Estimate distance
        distance_est = sqrt(K / P_r_distance);
        
        fprintf('P_r_distance: %.6f\n', P_r_distance);
        fprintf('Estimated distance: %.4f m (Real: %.4f m)\n', distance_est, real_distance);
        
        % Process direction estimation
        % Identify unique orientations
        orientaciones = unique(data(:, {'inclinacion', 'azimuth'}), 'rows');
        num_orientaciones = height(orientaciones);
        
        % Validate orientation indices
        K_valid = K_indices(K_indices >= 1 & K_indices <= num_orientaciones);
        if isempty(K_valid)
            warning('No valid orientation indices, using first 5');
            K_valid = 1:min(5, num_orientaciones);
        end
        
        % Create orientation matrix nt
        nt = zeros(3, length(K_valid));
        medidas_por_orientacion = {};
        max_samples = 0;
        
        % Process each selected orientation
        for idx = 1:length(K_valid)
            i = K_valid(idx);
            
            inclinacion = orientaciones.inclinacion(i);
            azimuth = orientaciones.azimuth(i);
            
            % Convert to unit vector
            theta = deg2rad(inclinacion);
            phi = deg2rad(azimuth);
            nt(:, idx) = [sin(theta)*cos(phi); sin(theta)*sin(phi); -cos(theta)];
            
            % Filter data for this orientation
            orientacion_data = data(data.inclinacion == inclinacion & ...
                                   data.azimuth == azimuth & ...
                                   strcmp(data.stage, 'direction'), :);
            
            if ~isempty(orientacion_data)
                medidas_por_orientacion{idx} = orientacion_data.medida_daq;
                max_samples = max(max_samples, height(orientacion_data));
            else
                medidas_por_orientacion{idx} = [];
            end
        end
        
        % Create Praw matrix
        Praw = zeros(max_samples, length(K_valid));
        for idx = 1:length(K_valid)
            medidas = medidas_por_orientacion{idx};
            if ~isempty(medidas)
                n_samples = length(medidas);
                Praw(1:n_samples, idx) = -(medidas - background_mean);
                if n_samples < max_samples
                    Praw(n_samples+1:end, idx) = Praw(n_samples, idx);
                end
            else
                Praw(:, idx) = NaN;
            end
        end
        
        % Remove orientations without data
        valid_data = ~any(isnan(Praw), 1);
        if sum(valid_data) < 3
            error('Not enough valid orientations (need at least 3)');
        end
        nt = nt(:, valid_data);
        Praw = Praw(:, valid_data);
        
        % Estimate direction vector
        d_hat = vlp_gls(nt, Praw, m);
        
        % Ensure unit vector
        d_hat = d_hat / norm(d_hat);
        
        % Calculate angular error
        dot_product = dot(direction_unit_vector, d_hat);
        dot_product = max(-1, min(1, dot_product));
        angular_error_deg = rad2deg(acos(dot_product));
        
        fprintf('Estimated direction: [%.4f, %.4f, %.4f]\n', d_hat(1), d_hat(2), d_hat(3));
        fprintf('Real----- direction: [%.4f, %.4f, %.4f]\n', direction_unit_vector(1), direction_unit_vector(2), direction_unit_vector(3));
        fprintf('Angular error: %.2f degrees\n', angular_error_deg);
        
        % Estimate 3D position
        position_est = transmitter_pos + distance_est * d_hat' + [0.13,0.088,0.087];
        
        
        fprintf('Estimated position: [%.4f, %.4f, %.4f]\n', position_est(1), position_est(2), position_est(3));
        
        % Store results
        reference_positions = [reference_positions; receiver_pos];
        estimated_positions = [estimated_positions; position_est];
        successful_files{end+1} = csv_files(file_idx).name;
        
    catch e
        fprintf('ERROR processing file %s: %s\n', csv_files(file_idx).name, e.message);
        failed_files{end+1} = csv_files(file_idx).name;
    end
end

% Display summary
fprintf('\n===========================================\n');
fprintf('PROCESSING SUMMARY\n');
fprintf('===========================================\n');
fprintf('Successfully processed: %d files\n', length(successful_files));
fprintf('Failed: %d files\n', length(failed_files));

if ~isempty(failed_files)
    fprintf('\nFailed files:\n');
    for i = 1:length(failed_files)
        fprintf('  - %s\n', failed_files{i});
    end
end
%%
% Create comparison figure
if ~isempty(reference_positions)
    figure('Position', [100, 100, 1400, 800]);
    
    % 3D scatter plot with improved aesthetics
    subplot(2, 2, 1);
    
    % Calculate position errors for color coding
    position_errors = vecnorm(reference_positions - estimated_positions, 2, 2);
    
    % Plot reference positions with consistent size
    scatter3(reference_positions(:,1), reference_positions(:,2), reference_positions(:,3), ...
             80, [0 0.4470 0.7410], 'o', 'filled', 'DisplayName', 'Reference', ...
             'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    hold on;
    
    % Plot estimated positions with error-based coloring
    scatter3(estimated_positions(:,1), estimated_positions(:,2), estimated_positions(:,3), ...
             80, position_errors, '^', 'filled', 'DisplayName', 'Estimated', ...
             'MarkerEdgeColor', 'k', 'LineWidth', 0.5);
    colormap(gca, 'hot');
    c = colorbar;
    c.Label.String = 'Error (m)';
    
    % Draw error lines with transparency
    for i = 1:size(reference_positions, 1)
        error_line = plot3([reference_positions(i,1), estimated_positions(i,1)], ...
                          [reference_positions(i,2), estimated_positions(i,2)], ...
                          [reference_positions(i,3), estimated_positions(i,3)], ...
                          'k-', 'LineWidth', 0.3);
        error_line.Color = [0.5 0.5 0.5 0.3]; % Gray with transparency
    end
    
    % Add transmitter position with distinctive marker
    scatter3(transmitter_pos(1), transmitter_pos(2), transmitter_pos(3), ...
             250, [0.4660 0.6740 0.1880], 'p', 'filled', 'DisplayName', 'Transmitter', ...
             'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % Aesthetics improvements
    xlabel('X (m)', 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontWeight', 'bold');
    zlabel('Z (m)', 'FontWeight', 'bold');
    title('3D Position Estimation Results', 'FontSize', 12);
    %legend('Location', 'northeast', 'FontSize', 9);
    grid on;
    grid minor;
    axis equal;
    view(45, 30);
    
    % Set axis limits with padding
    x_range = [min([reference_positions(:,1); estimated_positions(:,1); transmitter_pos(1)]), ...
               max([reference_positions(:,1); estimated_positions(:,1); transmitter_pos(1)])];
    y_range = [min([reference_positions(:,2); estimated_positions(:,2); transmitter_pos(2)]), ...
               max([reference_positions(:,2); estimated_positions(:,2); transmitter_pos(2)])];
    z_range = [min([reference_positions(:,3); estimated_positions(:,3); transmitter_pos(3)]), ...
               max([reference_positions(:,3); estimated_positions(:,3); transmitter_pos(3)])];
    padding = 0.2;
    xlim([x_range(1)-padding, x_range(2)+padding]);
    ylim([y_range(1)-padding, y_range(2)+padding]);
    zlim([z_range(1)-padding, z_range(2)+padding]);
    
    % Error analysis subplot - Histogram
    subplot(2, 2, 2);
    
    % Calculate position errors
    position_errors = vecnorm(reference_positions - estimated_positions, 2, 2);
    
    % Histogram of errors
    histogram(position_errors, 20, 'FaceColor', 'b', 'EdgeColor', 'k');
    xlabel('Position Error (m)');
    ylabel('Frequency');
    title('Position Error Distribution');
    grid on;
    
    % Calculate statistics including 90th percentile
    mean_error = mean(position_errors);
    std_error = std(position_errors);
    max_error = max(position_errors);
    min_error = min(position_errors);
    percentile_90 = prctile(position_errors, 90);
    
    text_str = sprintf('Mean Error: %.4f m\nStd Dev: %.4f m\nMin Error: %.4f m\nMax Error: %.4f m\n90th Percentile: %.4f m', ...
                      mean_error, std_error, min_error, max_error, percentile_90);
    text(0.05, 0.95, text_str, 'Units', 'normalized', ...
         'VerticalAlignment', 'top', 'FontSize', 9, ...
         'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    % CDF subplot
    subplot(2, 2, 3);
    
    % Sort errors for CDF
    sorted_errors = sort(position_errors);
    n_points = length(sorted_errors);
    cdf_values = (1:n_points) / n_points;
    
    % Plot CDF
    plot(sorted_errors, cdf_values, 'b-', 'LineWidth', 2);
    hold on;
    
    % Mark 90th percentile
    plot([percentile_90, percentile_90], [0, 0.9], 'r--', 'LineWidth', 1.5);
    plot([0, percentile_90], [0.9, 0.9], 'r--', 'LineWidth', 1.5);
    plot(percentile_90, 0.9, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % Add text annotation for 90th percentile
    text(percentile_90 + 0.01, 0.85, sprintf('90th: %.3f m', percentile_90), ...
         'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);
    
    xlabel('Position Error (m)');
    ylabel('Cumulative Probability');
    title('Cumulative Distribution Function (CDF)');
    grid on;
    xlim([0, max(sorted_errors) * 1.1]);
    ylim([0, 1]);
    
    % 2D Error Heatmap - Top View
    subplot(2, 2, 4);
    
    % Create grid for interpolation
    x_min = min(reference_positions(:,1)) - 0.2;
    x_max = max(reference_positions(:,1)) + 0.2;
    y_min = min(reference_positions(:,2)) - 0.2;
    y_max = max(reference_positions(:,2)) + 0.2;
    
    % Create fine grid for interpolation
    grid_resolution = 50;
    [X_grid, Y_grid] = meshgrid(linspace(x_min, x_max, grid_resolution), ...
                                linspace(y_min, y_max, grid_resolution));
    
    % Interpolate error values on grid
    try
        % Use scatteredInterpolant for smoother interpolation
        F = scatteredInterpolant(reference_positions(:,1), reference_positions(:,2), ...
                                position_errors, 'natural', 'linear');
        Z_grid = F(X_grid, Y_grid);
        
        % Plot heatmap
        imagesc([x_min x_max], [y_min y_max], Z_grid);
        set(gca, 'YDir', 'normal');
        colormap(gca, 'jet');
        colorbar_handle = colorbar;
        colorbar_handle.Label.String = 'Position Error (m)';
        caxis([min(position_errors), max(position_errors)]);
        
    catch
        % Fallback to simpler visualization if interpolation fails
        scatter(reference_positions(:,1), reference_positions(:,2), ...
                200, position_errors, 'filled', 's');
        colormap(gca, 'jet');
        colorbar_handle = colorbar;
        colorbar_handle.Label.String = 'Position Error (m)';
    end
    
    hold on;
    
    % Overlay reference positions
    scatter(reference_positions(:,1), reference_positions(:,2), ...
            50, 'w', 'o', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 1);
    
    % Add transmitter position
    scatter(transmitter_pos(1), transmitter_pos(2), ...
            200, 'w', 'p', 'filled', 'MarkerEdgeColor', 'k', 'LineWidth', 2);
    
    % Add contour lines for better visualization
    if exist('Z_grid', 'var')
        contour(X_grid, Y_grid, Z_grid, 5, 'k', 'LineWidth', 0.5, 'LineStyle', '--');
    end
    
    xlabel('X (m)', 'FontWeight', 'bold');
    ylabel('Y (m)', 'FontWeight', 'bold');
    title('2D Error Heatmap (Top View)', 'FontSize', 12);
    axis equal;
    grid on;
    grid minor;
    xlim([x_min, x_max]);
    ylim([y_min, y_max]);
    
    sgtitle('3D Position Estimation: Dataset Nikola Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    
    % Print final statistics
    fprintf('\n===========================================\n');
    fprintf('POSITION ESTIMATION STATISTICS\n');
    fprintf('===========================================\n');
    fprintf('Number of positions estimated: %d\n', size(reference_positions, 1));
    fprintf('Mean position error: %.4f m\n', mean_error);
    fprintf('Std deviation: %.4f m\n', std_error);
    fprintf('Min position error: %.4f m\n', min_error);
    fprintf('Max position error: %.4f m\n', max_error);
    fprintf('90th Percentile error: %.4f m\n', percentile_90);
    
    % Calculate errors per axis
    axis_errors = reference_positions - estimated_positions;
    fprintf('\nPer-axis errors:\n');
    fprintf('X-axis: Mean = %.4f m, Std = %.4f m\n', mean(abs(axis_errors(:,1))), std(axis_errors(:,1)));
    fprintf('Y-axis: Mean = %.4f m, Std = %.4f m\n', mean(abs(axis_errors(:,2))), std(axis_errors(:,2)));
    fprintf('Z-axis: Mean = %.4f m, Std = %.4f m\n', mean(abs(axis_errors(:,3))), std(axis_errors(:,3)));
    
else
    warning('No successful position estimations to display');
end

fprintf('\nAnalysis complete!\n');

% CALIBRATION - Calculates the constant K for the VLP system
% 
% This script reads the calibration data from CSV in dataset/calibration
% and calculates the constant K = d^2 * Pr, where:
% - d is the distance between transmitter (0,0,2) and receiver (x,y,z)
% - Pr is the corrected received power (measurement - background)

clear; clc;

% Define paths
calibration_dir = '../Database/data_calibration/';
calibration_file = 'data_0.000000_0.000000_0.900000.csv';
calibration_path = fullfile(calibration_dir, calibration_file);

% Verify that the file exists
if ~exist(calibration_path, 'file')
    error('Calibration file not found: %s', calibration_path);
end

% Read data from CSV
fprintf('Reading calibration data from: %s\n', calibration_path);
data = readtable(calibration_path);

% Extract coordinates from the filename
[~, filename, ~] = fileparts(calibration_file);
coords_str = strrep(filename, 'data_', '');
coords = sscanf(coords_str, '%f_%f_%f');

if length(coords) ~= 3
    error('Could not extract coordinates from the filename');
end

x_recv = coords(1);
y_recv = coords(2);
z_recv = coords(3);

fprintf('Receiver position: (%.6f, %.6f, %.6f)\n', x_recv, y_recv, z_recv);

% Transmitter position (fixed)
x_trans = 0;
y_trans = 0;
z_trans = 2;

% Calculate distance between transmitter and receiver
d = sqrt((x_recv - x_trans)^2 + (y_recv - y_trans)^2 + (z_recv - z_trans)^2);
fprintf('Transmitter-receiver distance: %.6f m\n', d);

% Filter data for inclination=0 and azimuth=0 with stage="direction"
direction_mask = (data.inclinacion == 0) & (data.azimuth == 0) & strcmp(data.stage, 'direction');
direction_data = data(direction_mask, :);

% Filter background data
background_mask = strcmp(data.stage, 'background');
background_data = data(background_mask, :);

if isempty(direction_data)
    warning('No direction data found with inclination=0 and azimuth=0');
    fprintf('Available stage types: %s\n', strjoin(unique(data.stage), ', '));
    fprintf('Unique inclination values: %s\n', num2str(unique(data.inclinacion)));
    fprintf('Unique azimuth values: %s\n', num2str(unique(data.azimuth)));
    
    % If there's no direction data, use only background for demonstration
    if ~isempty(background_data)
        background_mean = mean(background_data.medida_daq);
        fprintf('Only background data available. Mean: %.6f\n', background_mean);
        K = d^2 * background_mean; % This is only for demonstration
        fprintf('K calculated with background data: %.6f\n', K);
        return;
    else
        error('No valid data found for calibration');
    end
end

if isempty(background_data)
    error('No background data found for calibration');
end

% Calculate means
direction_mean = mean(direction_data.medida_daq);
background_mean = mean(background_data.medida_daq);

% Calculate corrected received power
Pr = - (direction_mean - background_mean);

% Calculate constant K
K_exp = d^2 * Pr;

% Show results
fprintf('\n--- Calibration Results ---\n');
fprintf('Number of direction measurements (0°,0°): %d\n', height(direction_data));
fprintf('Number of background measurements: %d\n', height(background_data));
fprintf('Direction power mean: %.6f\n', direction_mean);
fprintf('Background power mean: %.6f\n', background_mean);
fprintf('Corrected received power (Pr): %.6f\n', Pr);
fprintf('Distance (d): %.6f m\n', d);
fprintf('Calibration constant K = d² × Pr: %.6f\n', K_exp);

% Save result to file
save('calibration.mat', 'K_exp');

fprintf('\nResults saved in calibration_results.mat\n');
fprintf('Variable K available in workspace: %.6f\n', K_exp);

% Main script for reading CSV data and processing for direction estimation

% Configuration
clear;
close all;
clc;

% Constants
m = 3; % Lambertian order (adjust as needed)

% CSV file to read
% csv_file = '../dataset/data_0.000000_0.000000_1.000000.csv';
% csv_file = '../dataset/data_-0.600000_0.600000_1.200000.csv';
% csv_file = '../dataset/data_-0.600000_0.000000_1.200000.csv';
csv_file = '../dataset_RA_50K/data_-0.200000_-0.200000_1.100000.csv';
% csv_file = '../dataset/calibration/data_0.000000_0.000000_0.800000.csv';

% ======================================================================
% Vector of orientation indices to use (will be adjusted after identifying orientations)
% For example, K = [1,6,7,8,9] will select orientations at those positions
% K = [1,7,8,9,10]; % Default, empty means use all orientations
K = [1,3,4,5,6]; % Default, empty means use all orientations
% K = [1,3,4,5,6,7,8,9,10]
% ======================================================================

% Extract receiver position from CSV file name
[~, filename, ~] = fileparts(csv_file);
parts = split(filename, '_');
if length(parts) >= 4
    receiver_x = str2double(parts{2});
    receiver_y = str2double(parts{3});
    receiver_z = str2double(parts{4});
    receiver_pos = [receiver_x, receiver_y, receiver_z];
    transmitter_pos = [0, 0, 2]; % Fixed transmitter position
    
    % Calculate real unit direction vector (from transmitter to receiver)
    direction_vector = receiver_pos - transmitter_pos;
    direction_unit_vector = direction_vector / norm(direction_vector);
    
    disp(['Receiver position: [' num2str(receiver_x) ', ' num2str(receiver_y) ', ' num2str(receiver_z) ']']);
    disp(['Real direction vector (unit): [' num2str(direction_unit_vector(1)) ', ' num2str(direction_unit_vector(2)) ', ' num2str(direction_unit_vector(3)) ']']);
else
    warning('Could not extract receiver position from file name');
    direction_unit_vector = [];
end

% Read CSV file
disp(['Reading CSV file: ' csv_file]);
try
    data = readtable(csv_file);
    disp('CSV read successfully');
catch e
    error(['Error reading CSV: ' e.message]);
end

% Show CSV structure
disp('Data structure:');
disp(data(1:5,:));

% Identify unique orientations
disp('Identifying unique orientations...');
orientaciones = unique(data(:, {'inclinacion', 'azimuth'}), 'rows');
num_orientaciones = height(orientaciones);
disp(['Found ' num2str(num_orientaciones) ' different orientations.']);

% Display available orientations
disp('Available orientations:');
for i = 1:num_orientaciones
    disp([num2str(i) ': Inclination = ' num2str(orientaciones.inclinacion(i)) ...
         '°, Azimuth = ' num2str(orientaciones.azimuth(i)) '°']);
end

% Ask user for orientations to use if not defined
if isempty(K)
    disp('Using all available orientations');
    K = 1:num_orientaciones;
else
    % Validate that indices are within range
    K = K(K >= 1 & K <= num_orientaciones);
    if isempty(K)
        warning('Selected orientation indices are not valid. Using all orientations.');
        K = 1:num_orientaciones;
    else
        disp(['Using ' num2str(length(K)) ' selected orientations out of ' num2str(num_orientaciones) ' available.']);
    end
end

% Separate background data
background_data = data(strcmp(data.stage, 'background'), :);
disp(['Background data: ' num2str(height(background_data)) ' samples.']);

% Calculate mean and variance of background measurements
if ~isempty(background_data)
    background_values = background_data.medida_daq;
    background_mean = mean(background_values);
    background_var = var(background_values);
    disp(['Mean of background measurements: ' num2str(background_mean)]);
    disp(['Variance of background measurements: ' num2str(background_var)]);
    
    % Plot background signal in a separate figure
    figure;
    plot(1:length(background_values), background_values, 'b', 'MarkerSize', 8);
    hold on;
    plot([1, length(background_values)], [background_mean, background_mean], 'r-', 'LineWidth', 2);
    xlabel('Sample Number');
    ylabel('Measured Value');
    title('Background Signal');
    axis([-inf inf 0 1])
    legend('Background Measurements', ['Mean (' num2str(background_mean, '%.4f') ')']);
    grid on;
    hold off;
else
    warning('No background data found');
    background_mean = 0; % Default value if no background
    background_var = 0;
end

% Create matrix nt (orientation vectors) for selected orientations
nt = zeros(3, length(K));

% Create matrix to store measurements (Praw)
medidas_por_orientacion = {};
max_samples = 0;



% Process each selected orientation
for idx = 1:length(K)
    i = K(idx); % Index of selected orientation
    
    % Get angles of current orientation
    inclinacion = orientaciones.inclinacion(i);
    azimuth = orientaciones.azimuth(i);
    
    % Convert angles to orientation vector (Cartesian coordinates)
    % Inclination from vertical (theta) and azimuth from X axis (phi)
    theta = deg2rad(inclinacion);
    phi = deg2rad(azimuth);
    
    % Convert to unit vector (spherical to Cartesian coordinates)
    nt(:, idx) = [sin(theta)*cos(phi); sin(theta)*sin(phi); -cos(theta)];
    
    % Filter data for this orientation
    orientacion_data = data(data.inclinacion == inclinacion & ...
                            data.azimuth == azimuth & ...
                            strcmp(data.stage, 'direction'), :);
    
    % Save measurements for this orientation
    if ~isempty(orientacion_data)
        medidas_por_orientacion{idx} = orientacion_data.medida_daq;
        max_samples = max(max_samples, height(orientacion_data));
    else
        warning(['No samples found for orientation: inclination = ' ...
                 num2str(inclinacion) ', azimuth = ' num2str(azimuth)]);
        medidas_por_orientacion{idx} = [];
    end
    
    disp(['Orientation ' num2str(i) ' (index ' num2str(idx) ' of ' num2str(length(K)) '): Inclination = ' num2str(inclinacion) ...
         '°, Azimuth = ' num2str(azimuth) '° -> ' num2str(length(medidas_por_orientacion{idx})) ' samples.']);
end


% Create Praw matrix (N x n) for selected orientations
Praw = zeros(max_samples, length(K));
for idx = 1:length(K)
    medidas = medidas_por_orientacion{idx};
    if ~isempty(medidas)
        % Fill corresponding column with available measurements
        n_samples = length(medidas);
        Praw(1:n_samples, idx) = -(medidas - background_mean);
        
        % If there are fewer samples than maximum, fill with last value
        if n_samples < max_samples
            Praw(n_samples+1:end, idx) = medidas(end);
        end
    else
        % If no measurements for this orientation, fill with zeros or NaN
        Praw(:, idx) = NaN;
    end
end


% Verify if there is valid data for all orientations
valid_data = ~any(isnan(Praw), 1);
if ~all(valid_data)
    warning(['Missing data for ' num2str(sum(~valid_data)) ' orientations. Removing orientations without data.']);
    nt = nt(:, valid_data);
    Praw = Praw(:, valid_data);
    disp(['Processing with ' num2str(size(nt, 2)) ' valid orientations.']);
    
    % Check if there are enough orientations to continue
    if size(nt, 2) < 2
        error('At least 2 orientations are needed to estimate direction.');
    end
end

% Show matrix information
disp(['Dimension of nt: ' num2str(size(nt, 1)) 'x' num2str(size(nt, 2))]);
disp(['Dimension of Praw: ' num2str(size(Praw, 1)) 'x' num2str(size(Praw, 2))]);

% Visualize processed data (Praw) for each selected orientation
figure;
hold on;
colors = hsv(length(K)); % Different colors for each orientation
legendInfo = cell(length(K), 1);

disp('\nVariances of selected signals:');
disp('----------------------------------------');

for idx = 1:length(K)
    i = K(idx); % Original orientation index
    
    % Extract processed data from Praw for this orientation
    praw_data = Praw(:, idx);
    valid_data = ~isnan(praw_data);
    praw_data = praw_data(valid_data); % Remove NaN
    
    if ~isempty(praw_data)
        % Calculate mean and variance of processed data
        media_praw = mean(praw_data);
        var_praw = var(praw_data);
        
        % Show Praw variance in command window
        disp(['Praw Orientation ' num2str(i) ': Variance = ' num2str(var_praw, '%.6f')]);
        
        % Plot processed data for current orientation
        plot(1:length(praw_data), praw_data, 'Color', colors(idx,:), 'MarkerSize', 4);
        
        % Information for legend
        legendInfo{idx} = ['Ori. ' num2str(i) ': Inc=' num2str(orientaciones.inclinacion(i)) ...
                           '\circ, Az=' num2str(orientaciones.azimuth(i)) '\circ (media=' num2str(media_praw, '%.4f') ')'];
    else
        legendInfo{idx} = ['Ori. ' num2str(i) ': No data'];
    end
end
disp('----------------------------------------');
xlabel('Sample Number');
ylabel('Processed Value (Praw)');
title('Processed Data (Praw) by Selected Orientation');
grid on;
legend(legendInfo, 'Location', 'best');
hold off;

% Estimate direction using vlp_direction_cov_hetero
disp('Estimating vector direction...');
try
    %d_hat = vlp_direction_cov_hetero(nt, Praw, m);
    d_hat = vlp_gls(nt, Praw, m);
    % Show results
    disp('Estimated direction vector (from transmitter to receiver):');
    disp(d_hat);
    
    % Convert to angles for better interpretation
    [azimuth, elevation] = cart2sph(d_hat(1), d_hat(2), d_hat(3));
    azimuth = rad2deg(azimuth);
    elevation = rad2deg(elevation);
    
    disp(['Elevation: ' num2str(elevation) '°', 'Azimuth: ' num2str(azimuth) '°']);
    
    % Compare with real vector if available
    if ~isempty(direction_unit_vector)
        disp('\nComparison with real vector:');
        disp(['Real vector (unit): [' num2str(direction_unit_vector(1), '%.4f') ', ' num2str(direction_unit_vector(2), '%.4f') ', ' num2str(direction_unit_vector(3), '%.4f') ']']);
        disp(['Estimated vector: [' num2str(d_hat(1), '%.4f') ', ' num2str(d_hat(2), '%.4f') ', ' num2str(d_hat(3), '%.4f') ']']);
        
        % Calculate angular error
        dot_product = dot(direction_unit_vector, d_hat);
        % Ensure dot product is in range [-1, 1] to avoid numerical errors
        dot_product = max(-1, min(1, dot_product));
        angular_error = acos(dot_product);
        angular_error_deg = rad2deg(angular_error);
        
        disp(['Angular error: ' num2str(angular_error_deg, '%.2f') '°']);
        
        % Calculate Euclidean error
        euclidean_error = norm(direction_unit_vector - d_hat);
        disp(['Euclidean error: ' num2str(euclidean_error, '%.4f')]);
    end
    
    % Visualization
    figure;
    quiver3(0, 0, 0, d_hat(1), d_hat(2), d_hat(3), 'LineWidth', 2, 'Color', 'r');
    hold on;
    
    % Add real direction vector if available
    if ~isempty(direction_unit_vector)
        quiver3(0, 0, 0, direction_unit_vector(1), direction_unit_vector(2), direction_unit_vector(3), 'LineWidth', 2, 'Color', 'g');
    end
    
    % Visualize LED orientations
    for i = 1:size(nt, 2)
        quiver3(0, 0, 0, nt(1,i), nt(2,i), nt(3,i), 'LineWidth', 1, 'Color', 'b');
    end
    
    grid on;
    axis equal;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Comparison: Real Direction vs Estimated');
    if ~isempty(direction_unit_vector)
        legend('Estimated Direction', 'Real Direction', 'LED Orientations');
    else
        legend('Estimated Direction', 'LED Orientations');
    end
    axis([-1 1 -1 1 -1 0])
    
catch e
    error(['Error in direction estimation: ' e.message]);
end
clc, clear, close all

% Transmitter position
tx_pos = [0, 0, 2];

% PD FOV: 60 degrees from vertical
fov_angle = 60; % degrees
fov_threshold = cosd(fov_angle);

% Read positions from file
input_file = 'positions3D.txt';
output_file = 'positions3D_filtered.txt';

data = load(input_file);

fprintf('Total positions read: %d\n', size(data, 1));

% Visualization BEFORE filtering
figure('Name', 'Before FOV Filtering');
hold on;
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
view(45, 30);

% Plot transmitter
plot3(tx_pos(1), tx_pos(2), tx_pos(3), 'r^', 'MarkerSize', 15, ...
      'MarkerFaceColor', 'r', 'DisplayName', 'Transmitter');

% Plot all positions before filtering
plot3(data(:,1), data(:,2), data(:,3), ...
      'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'c', ...
      'DisplayName', 'All Positions');
legend('Location', 'best');
axis([-3 3 -3 3 0 2]);
hold off;

% Filter positions based on FOV
filtered_data = [];
count_inside = 0;
count_outside = 0;

for i = 1:size(data, 1)
    % Extract receiver position (first 3 elements)
    rx_pos = data(i, 1:3);
    
    % Calculate vector from transmitter to receiver
    vec_tx_to_rx = rx_pos - tx_pos;
    
    % Calculate distance
    distance = norm(vec_tx_to_rx);
    
    % Normalize the vector
    if distance > 0
        vec_normalized = vec_tx_to_rx / distance;
        
        % Vertical direction (pointing down from transmitter)
        vertical_down = [0, 0, -1];
        
        % Calculate cosine of angle between vectors
        cos_angle = dot(vec_normalized, vertical_down);
        
        % Check if within FOV (cos(angle) >= cos(60°))
        if cos_angle >= fov_threshold
            filtered_data = [filtered_data; data(i, :)];
            count_inside = count_inside + 1;
        else
            count_outside = count_outside + 1;
        end
    else
        % Position is at transmitter location, exclude it
        count_outside = count_outside + 1;
    end
end

% Display results
fprintf('\nFiltering results:\n');
fprintf('Positions inside FOV: %d\n', count_inside);
fprintf('Positions outside FOV: %d\n', count_outside);
fprintf('Percentage inside FOV: %.2f%%\n', (count_inside/size(data,1))*100);

% Save filtered data
dlmwrite(output_file, filtered_data, 'delimiter', ' ');
fprintf('\nFiltered data saved to: %s\n', output_file);

% Visualization AFTER filtering
figure('Name', 'After FOV Filtering');
hold on;
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
view(45, 30);

% Plot transmitter
plot3(tx_pos(1), tx_pos(2), tx_pos(3), 'r^', 'MarkerSize', 15, ...
      'MarkerFaceColor', 'r', 'DisplayName', 'Transmitter');

% Plot filtered positions (inside FOV)
if ~isempty(filtered_data)
    plot3(filtered_data(:,1), filtered_data(:,2), filtered_data(:,3), ...
          'bo', 'MarkerSize', 4, 'MarkerFaceColor', 'c', ...
          'DisplayName', 'Inside FOV');
end

% Plot rejected positions (outside FOV)
rejected_data = [];
for i = 1:size(data, 1)
    rx_pos = data(i, 1:3);
    vec_tx_to_rx = rx_pos - tx_pos;
    distance = norm(vec_tx_to_rx);
    
    if distance > 0
        vec_normalized = vec_tx_to_rx / distance;
        vertical_down = [0, 0, -1];
        cos_angle = dot(vec_normalized, vertical_down);
        
        if cos_angle < fov_threshold
            rejected_data = [rejected_data; data(i, :)];
        end
    end
end

if ~isempty(rejected_data)
    plot3(rejected_data(:,1), rejected_data(:,2), rejected_data(:,3), ...
          'rx', 'MarkerSize', 4, 'DisplayName', 'Outside FOV');
end

% Draw FOV cone
cone_height = 2; % from z=2 to z=0
cone_radius = cone_height * tand(fov_angle);
[X_cone, Y_cone, Z_cone] = cylinder([0, cone_radius], 50);
Z_cone = tx_pos(3) - Z_cone * cone_height;
X_cone = X_cone + tx_pos(1);
Y_cone = Y_cone + tx_pos(2);

surf(X_cone, Y_cone, Z_cone, 'FaceAlpha', 0.1, 'EdgeColor', 'none', ...
     'FaceColor', 'yellow', 'DisplayName', 'FOV Cone (60°)');

legend('Location', 'best');
axis([-3 3 -3 3 0 2])
hold off;

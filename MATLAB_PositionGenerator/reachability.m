%% Program to Process Points from a TXT File with 25 References
% Reads a file with data (X, Y, label). Then, for each of the 25 reference
% pairs (Xr, Yr), it evaluates all points with label 0.
% If for a given reference the point meets the conditions that its distance
% is within the range [min_distance, max_distance] and its angle (calculated 
% with respect to the Y-axis) is between [min_angle, max_angle],
% the point is assigned the iteration number (1 to 25) as its label.
% Finally, it plots:
% - Unlabeled points (label 0) as red circles.
% - Labeled points (label not 0) as dots with random colors.

clc; clear;

% Evaluation Parameters (ROBOT)
min_distance = 0.65;         % Required minimum distance
max_distance = 0.85;         % Required maximum distance
min_angle = 180;              % Minimum angle in degrees
max_angle = 360;             % Maximum angle in degrees
filename = 'positions3D.txt';   % Input file (ensure it is in the path)
% Load Data (POINTS)

data = load(filename);        % Expected to have three columns: [X, Y, label]
X = data(:,1);
Y = data(:,2);
labels = data(:,4);

% Define the 25 reference pairs (Xr, Yr) , (Robot's positions)
% ******************************************************************
% x_ref = [-0.5 ,0 ,0.5 ,1 ,1.5]; 
% y_ref = [-0.5 ,0 ,0.5 ,1 ,1.5];
% [X_ref, Y_ref] = meshgrid(x_ref, y_ref);
% references = [X_ref(:), Y_ref(:)];
references = [0 0.4];

% ******************************************************************

numPoints = size(references,1)
numRefs = numPoints;  % Number of references (should be 25)

% Processing: Evaluate each reference pair (Xr, Yr) sequentially
for i = 1:numRefs
    % Set the current reference position for this iteration
    Xr = references(i,1);
    Yr = references(i,2);
    
    % For each point that has not been labeled yet (label == 0)
    for j = 1:length(X)
        if labels(j) == 0
            % Calculate vector from the reference to the point
            dx = X(j) - Xr;
            dy = Y(j) - Yr;
            distance = sqrt(dx^2 + dy^2);
            
            % Calculate the angle between the vector and the Y-axis (in degrees)
            % Uses atan2 to obtain the correct angle. Multiplying -dx adjusts
            % the orientation relative to the Y-axis.
            if distance == 0
                angle = 0;
            else
                angle = mod(atan2(-dx, dy) * (180/pi), 360);
            end
            
            % Evaluate distance and angle conditions
            if (distance >= min_distance) && (distance <= max_distance) && (angle >= min_angle) && (angle <= max_angle)
                labels(j) = i;  % Assign the iteration number as the label
            end
        end
    end
end

% Update the data matrix with the modified labels
data(:,3) = labels;

numMissing = length(find(data(:,3)==0))

samples_per_position = [];
for k = 1:max(labels)
    samples_per_position = [samples_per_position; length(find(data(:,3)==k))];
end
samples_per_position

% Plotting
figure(1);

% Plot unlabeled points (label 0) as red circles
idx0 = labels == 0;
scatter(X(idx0), Y(idx0), 46, 'red');
hold on;

% Plot labeled points (label not 0) as dots with random colors
uniqueLabels = unique(labels(labels~=0));  % Unique assigned labels (1 to 16)
for k = 1:length(uniqueLabels)
    currentLabel = uniqueLabels(k);
    color = rand(1,3);  % Generate a random color
    idx = labels == currentLabel;
    % Plot using dot marker; using plot instead of scatter to change the marker style.
    plot(X(idx), Y(idx), '.', 'Color', color, 'MarkerSize', 60);
end

xlabel('X');
ylabel('Y');
title('Receiver positions sampled at each location of the robot platform');
grid minor;
grid on;
hold off;

% Save Results (Optional)
% Save the updated matrix in a new text file.
save('positions_labeled.txt', 'data', '-ascii');

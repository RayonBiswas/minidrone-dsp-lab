%% Route from phone to mini-drone waypoints

% Phone displays the map, START, GOAL and route.
% Laptop can mirror the phone's map and route.
% Export the route as a CSV file for the drone.
% The CSV file contains the corner points of the A* route.

% Drone starts at START facing up the map.
% x_fwd = forward, y_right = right, in metres.
% z is down, so flight height is represented by negative z.

clc; clear; close all;

% Set the CSV file name, flight height and waypoint spacing
fileName  = 'drone_route.csv';
height    = 1.1;      % m, flight height
spacing   = 0.20;     % m between waypoints after densifying (0 = corners only)


%% 1. Load route from CSV file

% Check whether the CSV file exists in the current folder
if ~isfile(fileName)

    % Search for the CSV file in the Windows Downloads folder
    dl = fullfile(getenv('USERPROFILE'), 'Downloads');

    % Search in both current folder and Downloads
    L = [dir('drone_route*.csv'); dir(fullfile(dl, 'drone_route*.csv'))];

    % Display an error if the file is not found
    if isempty(L), error('drone_route.csv not found. Tap "Route -> CSV" in the Navigate tab first.'); end

    % Select the most recently modified CSV file
    [~, iNew] = max([L.datenum]);
    fileName = fullfile(L(iNew).folder, L(iNew).name);
end

% Read CSV data while skipping the first four header lines
T = readmatrix(fileName, 'NumHeaderLines', 4);

% Extract forward and right coordinates from the CSV
corners = T(:, 4:5);

% Display the number of route corner points loaded
fprintf('Loaded %s: %d corner points\n', fileName, size(corners, 1));


%% 2. Densify the route to create evenly spaced waypoints

% Check whether waypoint spacing is enabled
if spacing > 0

    % Start with the first corner point
    wp = corners(1, :);

    % Process each pair of consecutive corner points
    for k = 2:size(corners, 1)

        % Get the starting and ending points of the route segment
        a = corners(k-1, :);
        b = corners(k, :);

        % Calculate the number of intermediate points needed
        n = max(1, ceil(norm(b - a) / spacing));

        % Generate evenly spaced points between the two corners
        wp = [wp; a + (1:n)' / n .* (b - a)]; %#ok<AGROW>
    end

else

    % Use only the original corner points
    wp = corners;
end

% Add flight height to every waypoint
% Negative z represents upward flight in NED coordinates
waypoints = [wp, -height * ones(size(wp, 1), 1)];

% Calculate the yaw angle for each route segment
% atan2 gives the heading angle in radians
yaw = [atan2(diff(wp(:,2)), diff(wp(:,1))); 0];

% Set the final yaw equal to the previous waypoint's yaw
yaw(end) = yaw(max(1, end-1));


% Display total waypoints and total route length
fprintf('%d waypoints, path length %.2f m\n', size(waypoints, 1), sum(vecnorm(diff(wp), 2, 2)));

% Display waypoint coordinates and heading in degrees
disp(array2table([waypoints, rad2deg(yaw)], 'VariableNames', {'x_fwd_m', 'y_right_m', 'z_m', 'yaw_deg'}));


%% 3. Plot the drone route from above

% Create a new figure for displaying the route
figure('Name', 'Drone route', 'Color', 'w');

% Apply light theme if available
if exist('theme', 'file') || exist('theme', 'builtin'), try, theme(gcf, 'light'); catch, end, end

% Plot all generated waypoints
plot(wp(:,2), wp(:,1), '.-', 'Color', [0.3 0.5 1], 'MarkerSize', 10);
hold on;

% Mark the original route corner points
plot(corners(:,2), corners(:,1), 'ko', 'MarkerSize', 8, 'LineWidth', 1.5);

% Mark the START and GOAL positions
plot(0, 0, 'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(wp(end,2), wp(end,1), 'rp', 'MarkerSize', 14, 'MarkerFaceColor', 'y');

% Set equal scaling and display the grid
axis equal;
grid on;

% Label both axes
xlabel('y right (m)');
ylabel('x forward (m)');

% Display the legend and plot title
legend('waypoints', 'route corners', 'START', 'GOAL', 'Location', 'best');
title('Route from the phone, drone frame');


%% 4. Use the generated route in Simulink

% Waypoints (N x 3) and yaw (N x 1) are available in the MATLAB workspace.
% They can be connected to the Path Planning subsystem.

% A MATLAB Function block can move to the next waypoint
% when the drone comes within approximately 0.1 m of the current waypoint.

% A From Workspace block can also be used to provide waypoint data.
% Example:
%   route = timeseries(waypoints, (0:size(waypoints,1)-1)' * 1.0);

% Save the generated waypoints, yaw angles and original corners
save('drone_waypoints.mat', 'waypoints', 'yaw', 'corners');

% Display confirmation message
fprintf('Saved drone_waypoints.mat (waypoints, yaw, corners).\n');

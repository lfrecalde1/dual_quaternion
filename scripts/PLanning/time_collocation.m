clc, clear all, close all;
% Define waypoints as a matrix, where each row is a waypoint (x, y, z)
waypoints = [0, 0, 0; 1, 2, 3; 4, 6, 8; 7, 8, 9];

% Total time for the trajectory
total_time = 10;  % Example total time

% Number of waypoints
num_waypoints = size(waypoints, 1);

% Calculate distances between consecutive waypoints
distances = zeros(num_waypoints - 1, 1);
for i = 1:(num_waypoints - 1)
    distances(i) = norm(waypoints(i + 1, :) - waypoints(i, :));
end

% Total distance
total_distance = sum(distances);

% Allocate time proportionally to each segment
time_segments = (distances / total_distance) * total_time;

% Generate the cumulative time vector
time = [0; cumsum(time_segments)];

% Display results
disp('Waypoints:');
disp(waypoints);
disp('Distances:');
disp(distances);
disp('Time segments:');
disp(time_segments);
disp('Cumulative time vector:');
disp(time);

% Plot the waypoints and time allocation for visualization
figure;
plot3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 'ro--', 'LineWidth', 2);
for i = 1:num_waypoints
    text(waypoints(i, 1), waypoints(i, 2), waypoints(i, 3), ['  t = ' num2str(time(i))]);
end
xlabel('X');
ylabel('Y');
zlabel('Z');
grid on;
title('Waypoints with Time Allocation');
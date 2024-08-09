%% Clean variables
clc, clear all, close all;

% Define waypoints along the x-axis
waypoints = [0; 2; 0];

% Total time for the trajectory
total_time = 2;  % Example total time

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

% Number of segments
N = length(waypoints) - 1;
t_init = time(1);
t_med = time(2);
t_final = time(3);

%% NUm coeff
numCoeff = 8;

%% Initial Time Constrainst
A_init= [position_time(t_init)', zeros(1, 8);...
    position_time(t_med)', zeros(1,8);...
    velocity_time(t_init)', zeros(1, 8);...
    acceleration_time(t_init)', zeros(1, 8)];

b_init = [waypoints(1); waypoints(2); 0;0;];
%% Constrainst Midle points velocity

A_v_m = [velocity_time(t_med)', zeros(1, 8);...
         zeros(1, 8), -velocity_time(t_med)'];

b_v_m = [0;-0];

%% Constrainst Middle Points Acceleration
A_a_m = [acceleration_time(t_med)', zeros(1, 8);...
         zeros(1, 8), -acceleration_time(t_med)'];

b_a_m = [0;0];

A_j_m = [jerk_time(t_med)', zeros(1, 8);...
         zeros(1, 8), -jerk_time(t_med)'];

b_j_m = [0;0];

%% Final Time Constrainsts
A_final = [zeros(1, 8), position_time(t_med)';...
    zeros(1, 8), position_time(t_final)';...
    zeros(1, 8), velocity_time(t_final)';...
    zeros(1, 8), acceleration_time(t_final)'];
b_final = [waypoints(2); waypoints(3); 0; 0];

A = [A_init; A_v_m; A_a_m; A_j_m; A_final];
b = [b_init; b_v_m; b_a_m; b_j_m; b_final];
%% Hessian
H_m = hessian_cost(t_med);
H_i = hessian_cost(t_init);
H_f = hessian_cost(t_final);
H_1 = H_m - H_i;
H_2 = H_f - H_m;

H = [H_1, zeros(8, 8);...
     zeros(8, 8), H_2];

% Solve the optimization problem
coeff_x = quadprog(H, [], [], [], A, b);


% % Plot the trajectory
figure;
hold on;
plot(time, waypoints, 'ro--', 'LineWidth', 2);
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    idx = (i - 1) * numCoeff + 1:i * numCoeff;
    px = coeff_x(idx)' * position_time(t);
    plot(t, px, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('px')
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
hold off;

figure;
hold on;
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    idx = (i - 1) * numCoeff + 1:i * numCoeff;
    vx = coeff_x(idx)' * velocity_time(t);
    plot(t, vx, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('vx');
grid on;
title('Minimum Snap Velocity between Two Waypoints');
hold off;


figure;
hold on;
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    idx = (i - 1) * numCoeff + 1:i * numCoeff;
    ax = coeff_x(idx)' * acceleration_time(t);
    plot(t, ax, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('ax');
grid on;
title('Minimum Snap acceleration between Two Waypoints');
hold off;

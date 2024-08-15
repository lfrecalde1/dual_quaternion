%% Clean variables
clc, clear all, close all;

% Define waypoints along the x-axis
waypoints = [0; 2; 0; 10];

% Total time for the trajectory
total_time = 30;  % Example total time

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
t_med1 = time(2);
t_med2 = time(3);
t_final = time(4);


%% NUm coeff
numCoeff = 10;

%% Initial Time Constrainst
A_init= [position_time(t_init)', zeros(1, 10), zeros(1, 10);...
    velocity_time(t_init)', zeros(1, 10), zeros(1, 10);...
    acceleration_time(t_init)', zeros(1, 10), zeros(1, 10);...
    jerk_time(t_init)', zeros(1, 10), zeros(1, 10)];

b_init = [waypoints(1);0;0;0];
%% Constrainst Midle points velocity

A_v_m1 = [position_time(t_med1)';...
    velocity_time(t_med1)';...
    acceleration_time(t_med1)';...
    jerk_time(t_med1)'];

A_v_m1_aux = [0  0  0  0 0 0 0 0, 0, 0;
            -velocity_time(t_init)';...
            -acceleration_time(t_init)';...
            -jerk_time(t_init)'];
A_zeros = zeros(4, 10);

A_v_m1 = [A_v_m1, A_v_m1_aux, A_zeros];
b_v_m1 = [waypoints(2);0;0;0];

A_v_m2 = [position_time(t_med2)';...
    velocity_time(t_med2)';...
    acceleration_time(t_med2)';...
    jerk_time(t_med2)'];

A_v_m2_aux = [0  0  0  0 0 0 0 0, 0, 0;
            -velocity_time(t_med1)';...
            -acceleration_time(t_med1)';...
            -jerk_time(t_med1)'];
A_zeros = zeros(4, 10);

A_v_m2 = [A_zeros, A_v_m2, A_v_m2_aux];
b_v_m2 = [waypoints(3);0;0;0];

A_v_m3 = [position_time(t_final)';...
    velocity_time(t_final)';...
    acceleration_time(t_final)';...
    jerk_time(t_final)'];

A_zeros = zeros(4, 10);

A_v_m3 = [A_zeros, A_zeros, A_v_m3];
b_v_m3 = [waypoints(4);0;0;0];

posi_2 = position_time(t_med1)';
posi_3 = position_time(t_med2)';
vector_pos = [zeros(1, 10), posi_2, zeros(1, 10)];
vector_pos2 = [zeros(1, 10), zeros(1, 10), posi_3];

A_velocity_m1 = [velocity_time(t_med1)', zeros(1, 10), zeros(1, 10);...
         zeros(1, 10), -velocity_time(t_med1)', zeros(1, 10)];

b_velocity_m1 = [0; -0];


%% Constrainst Middle Points Acceleration
A_a_m1 = [acceleration_time(t_med1)', zeros(1, 10), zeros(1, 10);...
         zeros(1, 10), -acceleration_time(t_med1)', zeros(1, 10)];

b_a_m1 = [0;0];

%% Constrainst Midle points velocity
A_velocity_m2 = [zeros(1, 10), velocity_time(t_med2)', zeros(1, 10);...
         zeros(1, 10), zeros(1, 10), -velocity_time(t_med2)'];

b_velocity_m2 = [-0;0];


A = [A_init; A_v_m1; A_v_m2; A_v_m3; vector_pos; vector_pos2]
b = [b_init; b_v_m1; b_v_m2; b_v_m3; waypoints(2); waypoints(3)]

% %% Hessian
H_m1 = hessian_cost(t_med1);
H_i = hessian_cost(t_init);

H_m2 = hessian_cost(t_med2);

H_f = hessian_cost(t_final);

H_1 = H_m1 - H_i;
H_2 = H_m2 - H_m1;
H_3 = H_f - H_m2;

H = [H_1, zeros(10, 10), zeros(10, 10);...
     zeros(10, 10), H_2, zeros(10, 10);...
     zeros(10, 10), zeros(10, 10), H_3];

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
ylabel('vx')
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
hold off;
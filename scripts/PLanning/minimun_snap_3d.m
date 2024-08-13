%% Minimun Snap 3D

%% Clean variables
clc, clear all, close all;

%% Times Trajectory
t_s = 0.05;
t_f = 20;
t = (0:t_s:t_f);

t_init_trajectory = t(50);
t_final_trajectory = t(end);

%% Time allocation for the two waypoints
time = [0, t_init_trajectory, t_final_trajectory];

%% Initial State
x_init = [0;0;5];
N = 2;

%% b vector inital point over trajectory and initial states of the system
zi = 2;
w_d = 3;
[b, b_final_trajectory] = trajectory_3d_init_final(zi, w_d, x_init, time(2), time(3));

%% Matrix A
A = A_matrix(time(1), time(2));

%% H matrix
[H] = H_matrix(time(1), time(2));

[coeff_x, f_val] = quadprog(H, [], [], [], A, b);

numCoeff = 10;
coeff_x_aux = reshape(coeff_x, numCoeff, 3);

%% Combine data
t_trajectory = (t_init_trajectory:t_s:t_final_trajectory);
t_initialization = (0:t_s:t_init_trajectory-t_s);
t_total = [t_initialization, t_trajectory];
%% Trajectory
[h_t, h_d_t, h_dd_t, h_ddd_t, h_dddd_t] = trajectory_3d(zi, w_d, t_trajectory);
[h_init, h_d_init, h_dd_init, h_ddd_init, h_dddd_init] = trajectory_3d_init_phase(coeff_x_aux, t_initialization);

%% Complete data
h = [h_init, h_t];
h_d = [h_d_init, h_d_t];
h_dd = [h_dd_init, h_dd_t];
h_ddd = [h_ddd_init, h_ddd_t];
h_dddd = [h_dddd_init, h_dddd_t];

figure;
hold on;
plot(t, h(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'x');
plot(t, h(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'y');
plot(t, h(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'z'); % Changed color for clarity
xlabel('Time');
ylabel('m');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;


figure;
hold on;
plot(t, h_d(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'vx');
plot(t, h_d(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'vy');
plot(t, h_d(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'vz'); % Changed color for clarity
xlabel('Time');
ylabel('m/s');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;

figure;
hold on;
plot(t, h_dd(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ax');
plot(t, h_dd(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'ay');
plot(t, h_dd(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'az'); % Changed color for clarity
xlabel('Time');
ylabel('m/s^2');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;


figure;
hold on;
plot(t, h_ddd(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'jx');
plot(t, h_ddd(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'jy');
plot(t, h_ddd(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'jz'); % Changed color for clarity
xlabel('Time');
ylabel('m/s^3');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;

figure;
hold on;
plot(t, h_dddd(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'sx');
plot(t, h_dddd(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'sy');
plot(t, h_dddd(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'sz'); % Changed color for clarity
xlabel('Time');
ylabel('m/s^4');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;
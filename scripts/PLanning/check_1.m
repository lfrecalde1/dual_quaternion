%% Clean variables
clc, clear all, close all;

%% Times Trajectory
t_s = 0.01;
t_f = 20;
t = (0:t_s:t_f);

t_init_trajectory = t(260);
t_final_trajectory = t(end);

%% Time allocation for the two segments: forward and reverse
time_forward = [0, t_init_trajectory];
time_reverse = [t_final_trajectory, t_final_trajectory + (t_final_trajectory - t_init_trajectory)];

%% Initial State
x_init = [0;0;0];

%% b vector for initial point to a certain point over the trajectory
zi = 2;
w_d = 2;
[b_forward] = trajectory_3d_init_final(zi, w_d, x_init, time_forward(2));

%% Matrix A and H for the forward trajectory
A_forward = A_matrix(time_forward(1), time_forward(2));
H_forward = H_matrix(time_forward(1), time_forward(2));

[coeff_x_forward, f_val_forward] = quadprog(H_forward, [], [], [], A_forward, b_forward);

%% Reshape coefficients
numCoeff = 10;
coeff_x_aux_forward = reshape(coeff_x_forward, numCoeff, 3);

%% Combine forward trajectory data
t_trajectory_forward = (t_init_trajectory:t_s:t_final_trajectory);
t_initialization = (0:t_s:t_init_trajectory-t_s);

%% Forward Trajectory
[h_t_forward, h_d_t_forward, h_dd_t_forward, h_ddd_t_forward, h_dddd_t_forward] = trajectory_3d(zi, w_d, t_trajectory_forward);
[h_init, h_d_init, h_dd_init, h_ddd_init, h_dddd_init] = trajectory_3d_init_phase(coeff_x_aux_forward, t_initialization);

%% Reverse Trajectory Optimization
[b_reverse] = trajectory_3d_init_final(zi, w_d, x_init, time_reverse(2));
A_reverse = A_matrix(time_reverse(1), time_reverse(2));
H_reverse = H_matrix(time_reverse(1), time_reverse(2));

[coeff_x_reverse, f_val_reverse] = quadprog(H_reverse, [], [], [], A_reverse, b_reverse);
coeff_x_aux_reverse = reshape(coeff_x_reverse, numCoeff, 3);

%% Reverse Trajectory
t_trajectory_reverse = (t_final_trajectory:-t_s:t_init_trajectory);
[h_t_reverse, h_d_t_reverse, h_dd_t_reverse, h_ddd_t_reverse, h_dddd_t_reverse] = trajectory_3d(zi, w_d, t_trajectory_reverse);

%% Complete data combining both forward and reverse trajectories
t_total = [t_initialization, t_trajectory_forward, t_trajectory_reverse];
h = [h_init, h_t_forward, h_t_reverse];
h_d = [h_d_init, h_d_t_forward, h_d_t_reverse];
h_dd = [h_dd_init, h_dd_t_forward, h_dd_t_reverse];
h_ddd = [h_ddd_init, h_ddd_t_forward, h_ddd_t_reverse];
h_dddd = [h_dddd_init, h_dddd_t_forward, h_dddd_t_reverse];

figure;
hold on;
plot(h(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'x');
plot(h(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'y');
plot(h(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'z'); % Changed color for clarity
xlabel('Time');
ylabel('m');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;
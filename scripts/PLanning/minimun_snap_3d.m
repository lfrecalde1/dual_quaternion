%% Minimun Snap 3D

%% Clean variables
clc, clear all, close all;

%% Times Trajectory
t_s = 0.01;
t_f = 20;
t_init = 2;
t = (t_init:t_s:t_f);

t_init_trajectory = t(500);
t_final_trajectory = t(end);

%% Time allocation for the two waypoints
time = [t_init, t_init_trajectory, t_final_trajectory, t_final_trajectory + 10];

%% Initial State
x_init = [5;5;5];
v_init = [1;2;-1];
a_init = [0;0;0];
j_init = [0;0;0];
s_init = [0;0;0];

%% b vector inital point over trajectory and initial states of the system
zi = 2;
w_d = 2;

%% Initial States second second
x = zi*cos(w_d*t_final_trajectory);
x_d = -zi*w_d*sin(w_d*t_final_trajectory);
x_dd = -zi*w_d*w_d*cos(w_d*t_final_trajectory);
x_ddd = zi*w_d*w_d*w_d*sin(w_d*t_final_trajectory);
x_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_final_trajectory);

%% Initial Points y
y = zi*sin(w_d*t_final_trajectory);
y_d = zi*w_d*cos(w_d*t_final_trajectory);
y_dd = -zi*w_d*w_d*sin(w_d*t_final_trajectory);
y_ddd = -zi*w_d*w_d*w_d*cos(w_d*t_final_trajectory);
y_dddd = zi*w_d*w_d*w_d*w_d*sin(w_d*t_final_trajectory);

%% Initial Point z
z = 1 + 0.1*t_final_trajectory;
z_d = 0.1;
z_dd = 0;
z_ddd = 0;
z_dddd = 0;

%% New Initial States
x_init_new = [x;y;z];
v_init_new = [x_d;y_d;z_d];
a_init_new = [x_dd;y_dd;z_dd];
j_init_new = [x_ddd;y_ddd;z_ddd];
s_init_new = [x_dddd;y_dddd;z_dddd];

[b] = trajectory_3d_init_final(zi, w_d, x_init, v_init, a_init, j_init, s_init, time(2));

[b1] = trajectory_3d_init_final_aux(zi, w_d, x_init_new, v_init_new, a_init_new, j_init_new, s_init_new, 0);

%% Matrix A
A = A_matrix(time(1), time(2));
A1 = A_matrix(time(3), time(4));
%% H matrix
[H] = H_matrix(time(1), time(2));
[H1] = H_matrix(time(3), time(4));

[coeff_x_1, f_val] = quadprog(H, [], [], [], A, b);
[coeff_x_2, f_val] = quadprog(2*H1, [], [], [], A1, b1);

numCoeff = 10;

coeff_x_aux_1 = reshape(coeff_x_1, numCoeff, 3);
coeff_x_aux_2 = reshape(coeff_x_2, numCoeff, 3);
%% Combine data
t_trajectory = (t_init_trajectory:t_s:t_final_trajectory);
t_initialization = (t_init:t_s:t_init_trajectory-t_s);
t_final = (t_final_trajectory:t_s:time(4));
t_total = [t_initialization, t_trajectory];

%% Trajectory
[h_t, h_d_t, h_dd_t, h_ddd_t, h_dddd_t] = trajectory_3d(zi, w_d, t_trajectory);
[h_init, h_d_init, h_dd_init, h_ddd_init, h_dddd_init] = trajectory_3d_init_phase(coeff_x_aux_1, t_initialization);
[h_final, h_d_final, h_dd_final, h_ddd_final, h_dddd_final] = trajectory_3d_init_phase(coeff_x_aux_2, t_final);

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
plot(t_final, h_final(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'x');
plot(t_final, h_final(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'y');
plot(t_final, h_final(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'z'); % Changed color for clarity
xlabel('Time');
ylabel('m');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;

% 
% figure;
% hold on;
% plot(t, h_d(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'vx');
% plot(t, h_d(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'vy');
% plot(t, h_d(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'vz'); % Changed color for clarity
% xlabel('Time');
% ylabel('m/s');
% grid on;
% title('Minimum Snap Trajectory between Two Waypoints');
% legend; % This will display the legend with the labels
% hold off;
% 
% figure;
% hold on;
% plot(t, h_dd(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'ax');
% plot(t, h_dd(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'ay');
% plot(t, h_dd(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'az'); % Changed color for clarity
% xlabel('Time');
% ylabel('m/s^2');
% grid on;
% title('Minimum Snap Trajectory between Two Waypoints');
% legend; % This will display the legend with the labels
% hold off;
% 
% 
% figure;
% hold on;
% plot(t, h_ddd(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'jx');
% plot(t, h_ddd(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'jy');
% plot(t, h_ddd(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'jz'); % Changed color for clarity
% xlabel('Time');
% ylabel('m/s^3');
% grid on;
% title('Minimum Snap Trajectory between Two Waypoints');
% legend; % This will display the legend with the labels
% hold off;
% 
% figure;
% hold on;
% plot(t, h_dddd(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'sx');
% plot(t, h_dddd(2, :), 'g-', 'LineWidth', 1.5, 'DisplayName', 'sy');
% plot(t, h_dddd(3, :), 'b-', 'LineWidth', 1.5, 'DisplayName', 'sz'); % Changed color for clarity
% xlabel('Time');
% ylabel('m/s^4');
% grid on;
% title('Minimum Snap Trajectory between Two Waypoints');
% legend; % This will display the legend with the labels
% hold off;
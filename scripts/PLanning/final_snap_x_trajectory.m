%% Clean variables
clc, clear all, close all;

%% Times Trajectory
t_s = 0.01;
t_f = 10;
t_init = 0;
t = (t_init:t_s:t_f);
t_init_trajectory = t(250);
t_mid_trajectory = t(750);
t_final_trajectory = t(end);

%% b vector inital point over trajectory and initial states of the system
zi = 2;
w_d = 2;

%% Initial States second second
x_init = zi*cos(w_d*t_init_trajectory);
x_init_d = -zi*w_d*sin(w_d*t_init_trajectory);
x_init_dd = -zi*w_d*w_d*cos(w_d*t_init_trajectory);
x_init_ddd = zi*w_d*w_d*w_d*sin(w_d*t_init_trajectory);
x_init_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_init_trajectory);

x_mid = zi*cos(w_d*t_mid_trajectory);
x_mid_d = -zi*w_d*sin(w_d*t_mid_trajectory);
x_mid_dd = -zi*w_d*w_d*cos(w_d*t_mid_trajectory);
x_mid_ddd = zi*w_d*w_d*w_d*sin(w_d*t_mid_trajectory);
x_mid_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_mid_trajectory);

%% Final States second second
x_final = zi*cos(w_d*t_final_trajectory);
x_final_d = -zi*w_d*sin(w_d*t_final_trajectory);
x_final_dd = -zi*w_d*w_d*cos(w_d*t_final_trajectory);
x_final_ddd = zi*w_d*w_d*w_d*sin(w_d*t_final_trajectory);
x_final_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_final_trajectory);

% Define waypoints along the x-axis
waypoints = [0; x_init; x_mid; x_final;];

% cumulative time vector
time = [0; t_init_trajectory; t_mid_trajectory; t_final_trajectory];
N = length(waypoints) - 1;
t_init = time(1);
t_med1 = time(2);
t_med2 = time(3);
tfinal = time(4);

%% NUm coeff
numCoeff = 10;

%% Initial Time Constrainst
A_init= [position_time(t_init)', zeros(1, 10), zeros(1, 10);...
    position_time(t_med1)', zeros(1,10), zeros(1, 10);...
    velocity_time(t_init)', zeros(1, 10), zeros(1, 10);...
    acceleration_time(t_init)', zeros(1, 10), zeros(1, 10)];

b_init = [waypoints(1); waypoints(2); 0; 0];
%% Constrainst Midle points velocity

A_v_m1 = [velocity_time(t_med1)', zeros(1, 10), zeros(1, 10);...
         zeros(1, 10), -velocity_time(t_med1)', zeros(1, 10)];

b_v_m1 = [0; -0];

%% Constrainst Middle Points Acceleration
A_a_m1 = [acceleration_time(t_med1)', zeros(1, 10), zeros(1, 10);...
         zeros(1, 10), -acceleration_time(t_med1)', zeros(1, 10)];

b_a_m1 = [0;-0];

%% Constrainst middle points jerk
A_j_m1 = [jerk_time(t_med1)', zeros(1, 10), zeros(1, 10);...
         zeros(1, 10), -jerk_time(t_med1)', zeros(1, 10)];

b_j_m1 = [0;-0];

%% Constrainsts positions middle point
A_x_m2= [zeros(1, 10), position_time(t_med1)', zeros(1, 10);...
         zeros(1, 10), position_time(t_med2)', zeros(1, 10)];

b_x_m2 = [waypoints(2); waypoints(3)];

%% Constrainst Midle points velocity
A_v_m2 = [zeros(1, 10), velocity_time(t_med2)', zeros(1, 10);...
         zeros(1, 10), zeros(1, 10), -velocity_time(t_med2)'];

b_v_m2 = [0;-0];

A_a_m2 = [zeros(1, 10), acceleration_time(t_med2)', zeros(1, 10);...
         zeros(1, 10), zeros(1, 10), -acceleration_time(t_med2)'];

b_a_m2 = [0;-0];

A_j_m2 = [zeros(1, 10), jerk_time(t_med2)', zeros(1, 10);...
         zeros(1, 10), zeros(1, 10), -jerk_time(t_med2)'];

b_j_m2 = [0;-0];

%% Final Time Constrainsts
A_final = [zeros(1, 10), zeros(1, 10), position_time(t_med2)';...
    zeros(1, 10), zeros(1, 10), position_time(tfinal)';...
    zeros(1, 10), zeros(1, 10), velocity_time(tfinal)';...
    zeros(1, 10), zeros(1, 10), acceleration_time(tfinal)'];
b_final = [waypoints(3); waypoints(4); 0; 0];
% 
A = [A_init; A_v_m1; A_a_m1; A_j_m1; A_x_m2; A_v_m2; A_a_m2; A_j_m2; A_final];
b = [b_init; b_v_m1; b_a_m1; b_j_m1; b_x_m2; b_v_m2; b_a_m2; b_j_m2; b_final];
% 
% %% Hessian
H_i = hessian_cost(t_init);

H_m1 = hessian_cost(t_med1);

H_m2 = hessian_cost(t_med2);

H_f = hessian_cost(tfinal);

H_1 = H_m1 - H_i;
H_2 = H_m2 - H_m1;
H_3 = H_f - H_m2;

H = [H_1, zeros(10, 10), zeros(10, 10);...
     zeros(10, 10), H_2, zeros(10, 10);...
     zeros(10, 10), zeros(10, 10), H_3];

%% Solve the optimization problem
coeff_x = quadprog(H, [], [], [], A, b);
% coeff_x = reshape(coeff_x, numCoeff, 3);
% coeff_c_init = coeff_x(:, 1);
% coeff_c_final = coeff_x(:, 3);

% % Plot the trajectory
figure;
hold on;
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
% 

% %% Time of each signal
% t_trajectory = (t_init_trajectory:t_s:t_final_trajectory);
% t_initialization = (t_init:t_s:t_init_trajectory-t_s);
% t_final = (t_final_trajectory+t_s:t_s:time(4));
% t_total = [t_initialization, t_trajectory, t_final];
% 
% %% All signals
% [h_t, h_d_t, h_dd_t, h_ddd_t, h_dddd_t] = trajectory_x(zi, w_d, t_trajectory);
% [h_init, h_d_init, h_dd_init, h_ddd_init, h_dddd_init] = trajectory_3d_init_phase(coeff_c_init, t_initialization);
% [h_final, h_d_final, h_dd_final, h_ddd_final, h_dddd_final] = trajectory_3d_init_phase(coeff_c_final, t_final);
% 
% %% Complete data
% h = [h_init, h_t, h_final];
% h_d = [h_d_init, h_d_t];
% h_dd = [h_dd_init, h_dd_t];
% h_ddd = [h_ddd_init, h_ddd_t];
% h_dddd = [h_dddd_init, h_dddd_t];
% 
% figure;
% hold on;
% plot(t_total, h(1, :), 'r-', 'LineWidth', 1.5, 'DisplayName', 'x');
% xlabel('Time');
% ylabel('m');
% grid on;
% title('Minimum Snap Trajectory between Two Waypoints');
% legend; % This will display the legend with the labels
% hold off;
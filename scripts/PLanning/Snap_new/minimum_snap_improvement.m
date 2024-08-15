%% Clean variables
clc, clear all, close all;

%% Define waypoints along the x-axis
waypoints_1 = [0.0; 2; 0; 2];
traj_size = size(waypoints_1, 1) -1;
ts = 0.02;

number_points = 1/ts;

%% Define time for ecah segment
traj_flight_times(1) = 1;
traj_flight_times(2) = 10;
traj_flight_times(3) = 1;

%% Build Matrices
A_zeros = zeros(4, 8);
A_zeros_aux = zeros(1, 8);
A_t_1 = A_matrix(traj_flight_times(1));
A_t_2 = A_matrix(traj_flight_times(2));
A_t_3 = A_matrix(traj_flight_times(3));
A_init = A_start();
A_med_aux = A_med();
A_pos = position_time(0)';

%% Sytem constraints velocity first segment
A_velocity_equality_1 = velocity_time(traj_flight_times(1))';
A_velocity_equality_2 = velocity_time(0)';

b_v1 = [0; 0];
%% System constraint acclerations
A_acce_equality_1 = acceleration_time(traj_flight_times(1))';
A_acc_equality_2 = acceleration_time(0)';

b_a1 = [0; 0];

%% Sytem constraints velocity second segment
A_velocity_equality_12 = velocity_time(traj_flight_times(2))';
A_velocity_equality_22 = velocity_time(0)';

b_v12 = [0; 0];

%% System constraint acclerations
A_acce_equality_12 = acceleration_time(traj_flight_times(2))';
A_acc_equality_22 = acceleration_time(0)';

b_a12 = [0;0];
%% Matrix A
% A = [A_init, A_zeros, A_zeros;...
%      A_t_1, A_med_aux, A_zeros;...
%      A_zeros, A_t_2, A_med_aux;...
%      A_zeros, A_zeros, A_t_3;...
%      A_zeros_aux, A_pos, A_zeros_aux;...
%      A_zeros_aux, A_zeros_aux, A_pos;...
%      A_velocity_equality_1, A_zeros_aux, A_zeros_aux;...
%      A_zeros_aux, A_velocity_equality_2, A_zeros_aux;...
%      A_acce_equality_1, A_zeros_aux, A_zeros_aux;...
%      A_zeros_aux, A_acc_equality_2, A_zeros_aux;...
%      A_zeros_aux, A_velocity_equality_12, A_zeros_aux;...
%      A_zeros_aux, A_zeros_aux, A_velocity_equality_22;...
%      A_zeros_aux, A_acce_equality_12, A_zeros_aux;...
%      A_zeros_aux, A_zeros_aux, A_acc_equality_22];

A = [A_init, A_zeros, A_zeros;...
     A_t_1, A_med_aux, A_zeros;...
     A_zeros, A_t_2, A_med_aux;...
     A_zeros, A_zeros, A_t_3;...
     A_zeros_aux, A_pos, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_pos];

%% Positions, velocities and accelerations
b_1 = [waypoints_1(1);0;0;0];
b_2 = [waypoints_1(2);0;0;0];
b_3 = [waypoints_1(3);0;0;0];
b_4 = [waypoints_1(4);0;0;0];
b_5 = [waypoints_1(2);waypoints_1(3)];

b =[b_1; b_2; b_3; b_4; b_5];

%% Hessian
H_f_1 = hessian_cost(traj_flight_times(1));
H_f_2 = hessian_cost(traj_flight_times(2));
H_f_3 = hessian_cost(traj_flight_times(3));
H_i = hessian_cost(0);

H1 = H_f_1- H_i;
H2 = H_f_2 - H_i;
H3 = H_f_3 - H_i;
H = blkdiag(H1, H2, H3);

traj_polys = quadprog(H, [], [], [], A, b);
coeff = reshape(traj_polys, 8, 3)'   

index = 0;
%% PLot results
POS = [];
VEL = [];
ACC = [];
TIME = [];
for k=1:traj_size
    plot_time = traj_flight_times(k)*number_points;
    time_step = traj_flight_times(k)/plot_time;

    for j=1:plot_time
        position(index + j) = coeff(k, :)*position_time((j - 1)*time_step);
        position_aux(k, index + j) = coeff(k, :)*position_time((j - 1)*time_step);
        velocity(index + j) = coeff(k, :)*velocity_time((j - 1)*time_step);
        acceleration(index + j) = coeff(k, :)*acceleration_time((j - 1)*time_step);
        jerk(index + j) = coeff(k, :)*jerk_time((j - 1)*time_step);
        snap(index + j) = coeff(k, :)*snap_time((j - 1)*time_step);
        time_data(index + j) =  (index + j - 1)*time_step;
    end
    
    index = index + plot_time;

end
%% Time definition
time = (0:ts:(size(position,2)*time_step)-ts);

figure;
hold on;
plot(time, position', 'r-', 'LineWidth', 1.5, 'DisplayName', 'x');
xlabel('Time');
ylabel('m');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;


figure;
hold on;
plot(time, velocity, 'r-', 'LineWidth', 1.5, 'DisplayName', 'vx');
xlabel('Time');
ylabel('m/s');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;

figure;
hold on;
plot(time, acceleration, 'r-', 'LineWidth', 1.5, 'DisplayName', 'ax');
xlabel('Time');
ylabel('m/s^{2}');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;


figure;
hold on;
plot(time, jerk, 'r-', 'LineWidth', 1.5, 'DisplayName', 'jx');
xlabel('Time');
ylabel('m/s^{3}');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;

figure;
hold on;
plot(time, snap, 'r-', 'LineWidth', 1.5, 'DisplayName', 'sx');
xlabel('Time');
ylabel('m/s^{4}');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;
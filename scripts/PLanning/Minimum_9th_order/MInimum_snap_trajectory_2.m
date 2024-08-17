%% Clean variables
clc, clear all, close all;

%% Define time for each segment
traj_flight_times(1) = 2;
traj_flight_times(2) = 10;
traj_flight_times(3) = 2;
traj_flight_times(4) = 2;

% %% Define waypoints along the x-axis
[h_init] = trajectory_time(traj_flight_times(1));
[h_final] = trajectory_time(traj_flight_times(2)+traj_flight_times(1));
% 
waypoints_1 = [0.0; h_init(1); h_final(1); 0; 0];
traj_size = size(waypoints_1, 1) -1;
ts = 0.01;
% 
number_points = 1/ts;
number_poly = 9;
number_coeff = number_poly + 1;
% 
t_trajectory = (traj_flight_times(1):ts:traj_flight_times(2) + traj_flight_times(1)-ts);
% %% Auxiliary variable time
traj_flight_times_aux(1) = 0.0;
traj_flight_times_aux(2) = traj_flight_times(1);
traj_flight_times_aux(3) = traj_flight_times(2);
traj_flight_times_aux(4) = traj_flight_times(3);
traj_flight_times_aux(5) = traj_flight_times(4);

for k=1:length(traj_flight_times_aux)
    aux_time(k) = sum(traj_flight_times_aux(1:k));
end

% %% Build Matrices
A_zeros = zeros(5, 10);
A_zeros_aux = zeros(1, 10);
A_t_1 = A_matrix(traj_flight_times(1));
A_t_2 = A_matrix(traj_flight_times(2));
A_t_3 = A_matrix(traj_flight_times(3));
A_t_4 = A_matrix(traj_flight_times(4));
A_init = A_start();
A_med_aux = A_med();
A_pos = position_time(0)';
% 
%% Constraints velocity acceleration jerk and snap final first segment
A_vel_equality_1 = velocity_time(traj_flight_times(1))';
A_acc_equality_1 = acceleration_time(traj_flight_times(1))';
A_jerk_equality_1 = jerk_time(traj_flight_times(1))';
A_snap_equality_1 = snap_time(traj_flight_times(1))';
% 
%% Final desired velocities first segment
b_first = [h_init(2);h_init(3);h_init(4);h_init(5)];
% 
%% Constraints velocity acceleration jerk and snap final second segment
A_vel_equality_2 = velocity_time(traj_flight_times(2))';
A_acc_equality_2 = acceleration_time(traj_flight_times(2))';
A_jerk_equality_2 = jerk_time(traj_flight_times(2))';
A_snap_equality_2 = snap_time(traj_flight_times(2))';

%% Final desired velocities second segment
b_second = [h_final(2);h_final(3);h_final(4);h_final(5)];
% 

%% Constraints velocity acceleration jerk and snap init third segment
A_vel_equality_3 = velocity_time(0)';
A_acc_equality_3 = acceleration_time(0)';
A_jerk_equality_3 = jerk_time(0)';
A_snap_equality_3 = snap_time(0)';

%% Final desired velocities third segment
b_third = b_second;

%% Constraints velocity acceleration jerk and snap init third segment
A_vel_equality_4 = velocity_time(0)';
A_acc_equality_4 = acceleration_time(0)';
A_jerk_equality_4 = jerk_time(0)';
A_snap_equality_4 = snap_time(0)';

%% Final desired velocities third segment
b_fourth = [0;0;0;0];

%% 
% 
% 
% %% Matrix A
A = [A_init, A_zeros, A_zeros, A_zeros;...
     A_t_1, A_med_aux, A_zeros, A_zeros;...
     A_zeros, A_t_2, A_med_aux, A_zeros;...
     A_zeros, A_zeros, A_t_3, A_med_aux;...
     A_zeros, A_zeros, A_zeros, A_t_4;...
     A_zeros_aux, A_pos, A_zeros_aux, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux,  A_pos, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_zeros_aux, A_pos;...
     A_vel_equality_1, A_zeros_aux, A_zeros_aux, A_zeros_aux;...
     A_acc_equality_1, A_zeros_aux, A_zeros_aux, A_zeros_aux;...
     A_jerk_equality_1, A_zeros_aux, A_zeros_aux, A_zeros_aux;...
     A_snap_equality_1, A_zeros_aux, A_zeros_aux, A_zeros_aux;...
     A_zeros_aux, A_vel_equality_2, A_zeros_aux, A_zeros_aux;...
     A_zeros_aux, A_acc_equality_2, A_zeros_aux, A_zeros_aux;...
     A_zeros_aux, A_jerk_equality_2, A_zeros_aux, A_zeros_aux;...
     A_zeros_aux, A_snap_equality_2, A_zeros_aux, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_vel_equality_3, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_acc_equality_3, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_jerk_equality_3, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_snap_equality_3, A_zeros_aux;...
     A_zeros_aux, A_zeros_aux, A_zeros_aux, A_vel_equality_4;...
     A_zeros_aux, A_zeros_aux, A_zeros_aux, A_acc_equality_4;...
     A_zeros_aux, A_zeros_aux, A_zeros_aux, A_jerk_equality_4;...
     A_zeros_aux, A_zeros_aux, A_zeros_aux, A_snap_equality_4];
% % 
% % %% Positions, velocities and accelerations
b_1 = [waypoints_1(1);0;0;0;0];
b_2 = [waypoints_1(2);0;0;0;0];
b_3 = [waypoints_1(3);0;0;0;0];
b_4 = [waypoints_1(4);0;0;0;0];
b_5 = [waypoints_1(5);0;0;0;0];
b_6 = [waypoints_1(2);waypoints_1(3);waypoints_1(4)];
% % % 
b =[b_1; b_2; b_3; b_4; b_5; b_6; b_first; b_second; b_second; b_fourth];
% % % 
% % % %% Hessian
H_f_1 = hessian_cost(traj_flight_times(1));
H_f_2 = hessian_cost(traj_flight_times(2));
H_f_3 = hessian_cost(traj_flight_times(3));
H_f_4 = hessian_cost(traj_flight_times(4));
H_i = hessian_cost(0);

H1 = H_f_1- H_i;
H2 = H_f_2 - H_i;
H3 = H_f_3 - H_i;
H4 = H_f_4 - H_i;
H = blkdiag(H1, H2, H3, H4);

traj_polys = quadprog(H, [], [], [], A, b);
coeff = reshape(traj_polys, number_coeff, traj_size)';
% 

index = 0;
%% PLot results

for k=1:traj_size
    plot_time = traj_flight_times(k)*number_points
    time_step = traj_flight_times(k)/plot_time
    if k ~= 2
        for j=1:plot_time
            (j - 1)*time_step
            position(index + j) = coeff(k, :)*position_time((j - 1)*time_step);
            velocity(index + j) = coeff(k, :)*velocity_time((j - 1)*time_step);
            acceleration(index + j) = coeff(k, :)*acceleration_time((j - 1)*time_step);
            jerk(index + j) = coeff(k, :)*jerk_time((j - 1)*time_step);
            snap(index + j) = coeff(k, :)*snap_time((j - 1)*time_step);
        end
    else

        for j=1:length(t_trajectory)
            h = trajectory_time(t_trajectory(j));
            position(index + j) = h(1);
            velocity(index + j) = h(2);
            acceleration(index + j) = h(3);
            jerk(index + j) = h(4);
            snap(index + j) = h(5);
        end
    end

    index = index + plot_time;

end
%% Time definition
time = (0:ts:(size(position,2)*time_step)-ts);

figure;
hold on;
plot(aux_time, waypoints_1, 'ro--', 'LineWidth', 2);
plot(time, position', 'r-', 'LineWidth', 1.5, 'DisplayName', 'x');
xlabel('Time');
ylabel('m');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
legend; % This will display the legend with the labels
hold off;

% 
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
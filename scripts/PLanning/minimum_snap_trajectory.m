%% Minimum snap planning
%% Clean variables
clc, clear all, close all;
%% Times Trajectory
t_init_trajectory =7;
t_final_trajectory = 20;

%% Trajectory
zi = 1;
w_d = 2;

%% initial Points
x = zi*cos(w_d*t_init_trajectory);
x_d = -zi*w_d*sin(w_d*t_init_trajectory);
x_dd = -zi*w_d*w_d*cos(w_d*t_init_trajectory);
x_ddd = zi*w_d*w_d*w_d*sin(w_d*t_init_trajectory);
x_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_init_trajectory);

%% Final Points
x_f = zi*cos(w_d*t_final_trajectory);

%% Define waypoints along the x-axis
waypoints = [0; x(1, 1); x_f(1, 1)];

% Number of segments
N = length(waypoints) - 1;

%% Time allocation for the two waypoints
time = [0, t_init_trajectory, t_final_trajectory];
t_init = time(1);
t_final = time(2);

%% Optimization Parameters
numCoeff = 10;
%% Equality Constrainst
A = [position_time(t_init)';...
     velocity_time(t_init)';...
     acceleration_time(t_init)';...
     jerk_time(t_init)';
     snap_time(t_init)';
     position_time(t_final)';...
     velocity_time(t_final)';...
     acceleration_time(t_final)'
     jerk_time(t_final)'
     snap_time(t_final)'];

b = [waypoints(1);0;0;0;0;waypoints(2);x_d;x_dd;x_ddd;x_dddd];

%% Hessian
H_f = hessian_cost(t_final);
H_i = hessian_cost(t_init);

H = H_f - H_i;
% Solve the optimization problem
[coeff_x, f_val] = quadprog(H, [], [], [], A, b)

%% Plot the trajectory
figure;
hold on;
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    if i == 1
        idx = (i - 1) * numCoeff + 1:i * numCoeff;
        px = coeff_x(idx)' * position_time(t);
    else
        px = zi*cos(w_d*t);
    end
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
    if i == 1
        idx = (i - 1) * numCoeff + 1:i * numCoeff;
        vx = coeff_x(idx)' * velocity_time(t);
    else
        vx = -zi*w_d*sin(w_d*t);
    end
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
    if i == 1
        idx = (i - 1) * numCoeff + 1:i * numCoeff;
        ax = coeff_x(idx)' * acceleration_time(t);
    else
        ax = -zi*w_d*w_d*cos(w_d*t);
    end
    plot(t, ax, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('ax');
grid on;
title('Minimum Snap acceleration between Two Waypoints');
hold off;


figure;
hold on;
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    if i == 1
        idx = (i - 1) * numCoeff + 1:i * numCoeff;
        jx = coeff_x(idx)' * jerk_time(t);
    else
        jx = zi*w_d*w_d*w_d*sin(w_d*t);
    end
    plot(t, jx, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('jx');
grid on;
title('Minimum Snap jerk between Two Waypoints');
hold off;


figure;
hold on;
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    if i == 1
        idx = (i - 1) * numCoeff + 1:i * numCoeff;
        sx = coeff_x(idx)' * snap_time(t);
    else
        sx = zi*w_d*w_d*w_d*w_d*cos(w_d*t);
    end
    plot(t, sx, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('sx');
grid on;
title('Minimum Snap  between Two Waypoints');
hold off;

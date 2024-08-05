%% Clean variables
clc, clear all, close all;

% Define waypoints along the x-axis
waypoints = [0; 1];

% Time allocation for the two waypoints
time = [0, 1];
t_init = time(1);
t_final = time(2);

%% Equality Constrainst
A = [position_time(t_init)';...
     velocity_time(t_init)';...
     acceleration_time(t_init)';...
     position_time(t_final)';...
     velocity_time(t_final)';...
     acceleration_time(t_final)'];

b = [waypoints(1);0;0;waypoints(2);0;0];

%% Hessian
H_f = hessian_cost(t_final);
H_i = hessian_cost(t_init);

H = H_f - H_i;
% Solve the optimization problem
coeff_x = quadprog(H, [], [], [], A, b);

%% PLot Results
t = linspace(t_init, t_final, 100);
px = coeff_x' * position_time(t);
vx = coeff_x' * velocity_time(t);
ax = coeff_x' * acceleration_time(t);
jx = coeff_x' * jerk_time(t);

figure
plot(t, px, 'b-', 'LineWidth', 1.5);
xlabel('Time');
ylabel('px');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
hold off;

figure
plot(t, vx, 'b-', 'LineWidth', 1.5);
xlabel('Time');
ylabel('vx');
grid on;
title('Minimum Snap Velocity between Two Waypoints');
hold off;

figure
plot(t, ax, 'b-', 'LineWidth', 1.5);
xlabel('Time');
ylabel('ax');
grid on;
title('Minimum Snap acceleration between Two Waypoints');
hold off;

figure
plot(t, jx, 'b-', 'LineWidth', 1.5);
xlabel('Time');
ylabel('jx');
grid on;
title('Minimum Snap jerk between Two Waypoints');
hold off;
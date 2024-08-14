%% Snap Optimizing time 
clc, clear all, close all;

%% CasADi v3.4.
addpath('/home/fer/casadi-3.6.6-linux64-matlab2018b');
import casadi.*;

%% Opt variables
numCoeff = 10;

%% Times Trajectory
t_s = 0.05;
t_f = 30;
t = (0:t_s:t_f);



%% Initial State
x_init = [10;10;10];

%% b vector inital point over trajectory and initial states of the system
zi = 2;
w_d = 2;
%% Symbolic time 
k = SX.sym('k', 1);
t_1_expr = k * t_s;

[b] = trajectory_3d_init_final(zi, w_d, x_init, t_1_expr);

%% Matrix A
A = A_matrix(0, t_1_expr);

%% H matrix
[H] = H_matrix(0, t_1_expr);

%% Optimization variables
coeff_x = SX.sym('coeff_x', numCoeff*3, 1);

%% Quadratic cost
lambda = 50500;
objective = coeff_x(:)' * H * coeff_x(:) + lambda * t_1_expr;

% maximum and minimum acceleration
a_max = 2.0;  % Maximum acceleration in m/s^2 (adjust as needed)
a_min = -2.0;  % Minimum acceleration in m/s^2 (adjust as needed)

constraints_aux1 = coeff_x(1:10,1)'*acceleration_time(t_1_expr) - a_max;
constraints_aux2 = -coeff_x(1:10,1)'*acceleration_time(t_1_expr) + a_min;
constraints_aux = [constraints_aux1; constraints_aux2];
% Define the equality constraints
constraints_eq = A * coeff_x(:) - b;

constraints = [constraints_eq; -k + 1];

% Set up the NLP problem in CasADi
nlp = struct('x', [coeff_x(:); k], 'f', objective, 'g', constraints);

% Create an NLP solver (IPOPT is a good general-purpose solver)
opts = struct;
opts.ipopt.max_iter = 5000;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp, opts);

% % Provide an initial guess for the solver (all zeros in this case)
coeff_init_guess = zeros(numCoeff*3, 1);
k_init_guess = round(1 / t_s);  % Initial guess for k
x0 = [coeff_init_guess(:); k_init_guess];

% Solve the problem
sol = solver('x0', [coeff_init_guess; k_init_guess], 'lbg', [zeros(length(constraints_eq),1);-inf], ...
            'ubg', [zeros(length(constraints_eq),1);0]);

% Extract optimized coefficients and t_1
coeff_opt = full(sol.x(1:end-1));
k_opt = full(sol.x(end));
t_1_opt = round(k_opt) * t_s
coeff_x_aux = reshape(coeff_opt, numCoeff, 3);
% 
% %% Combine data
%% Combine data
t_trajectory = (t_1_opt:t_s:t_f);
t_initialization = (0:t_s:t_1_opt-t_s);
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

% 
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
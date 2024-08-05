clc, clear all, close all
% Define waypoints along the x-axis
waypoints = [0; 1; 4];

% Time allocation (assuming equal time intervals for simplicity)
time = linspace(0, 10, length(waypoints));

% Number of segments
N = length(waypoints) - 1;

% Degree of the polynomial (7th degree for minimum snap)
deg = 7;

% Total number of coefficients per segment
numCoeff = deg + 1;

% Preallocate matrices for constraints
A = [];
b = [];
t0 = time(1);
t1 = time(2);
t2 = time(3);
% Initial and final position constraints for segment i
A_pos_i = [polyT(t0, deg); polyT(t1, deg)];
zeros_values = zeros(2, 8);
b_pos_i = [waypoints(1); waypoints(2)];

A = [A_pos_i, zeros_values];
b = [b;b_pos_i];

A_pos_aux = [polyTdot(t1, deg), zeros(1, 8)];
A_pos_aux_i = [zeros(1, 8), -polyTdot(t1, deg)];

A = [A; A_pos_aux; A_pos_aux_i];
b = [b;0;0];

A_pos_aux = [polyTddot(t1, deg), zeros(1, 8)];
A_pos_aux_i = [zeros(1, 8), -polyTddot(t1, deg)];

A = [A; A_pos_aux; -A_pos_aux_i];
b = [b;0;0];

A_pos_i = [polyT(t1, deg); polyT(t2, deg)];
b_pos_i = [waypoints(2); waypoints(3)];
A = [A; zeros(2, 8), A_pos_i]
b = [b; b_pos_i]

% Build cost matrix H for minimum snap (integral of snap squared)
H = zeros(numCoeff * N);
for i = 1:N
    H_i = polyH(time(i), time(i + 1), deg);
    idx = (i - 1) * numCoeff + 1:i * numCoeff;
    H(idx, idx) = H_i;
end
% 
% Solve the optimization problem
coeff_x = quadprog(H, [], [], [], A, b);
% 
% 
% Plot the trajectory
figure;
hold on;
plot(time, waypoints, 'ro--', 'LineWidth', 2);
for i = 1:N
    t = linspace(time(i), time(i + 1), 100);
    idx = (i - 1) * numCoeff + 1:i * numCoeff;
    x = polyval(flip(coeff_x(idx)'), t);
    plot(t, x, 'b-', 'LineWidth', 1.5);
end
xlabel('Time');
ylabel('X');
grid on;
title('Minimum Snap Trajectory');
hold off;
% % Function to generate the polynomial matrix
function T = polyT(t, deg)
    T = zeros(1, deg + 1);
    for i = 0:deg
        T(i + 1) = t^i;
    end
end
% 
% % Function to generate the derivative of the polynomial matrix
function T_dot = polyTdot(t, deg)
    T_dot = zeros(1, deg + 1);
    for i = 1:deg
        T_dot(i + 1) = i * t^(i - 1);
    end
end
% 
% Function to generate the second derivative of the polynomial matrix
function T_ddot = polyTddot(t, deg)
    T_ddot = zeros(1, deg + 1);
    for i = 2:deg
        T_ddot(i + 1) = i * (i - 1) * t^(i - 2);
    end
end
% 
% Function to generate the third derivative of the polynomial matrix
function T_dddot = polyTdddot(t, deg)
    T_dddot = zeros(1, deg + 1);
    for i = 3:deg
        T_dddot(i + 1) = i * (i - 1) * (i - 2) * t^(i - 3);
    end
end
% 
% Function to generate the cost matrix for minimum snap
function H = polyH(t0, t1, deg)
    H = zeros(deg + 1);
    for i = 4:deg
        for j = 4:deg
            H(i + 1, j + 1) = (i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) * (j - 3) / (i + j - 7)) * (t1^(i + j - 7) - t0^(i + j - 7));
        end
    end
end

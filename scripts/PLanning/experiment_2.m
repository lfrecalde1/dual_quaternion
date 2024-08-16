%% Clean variables
clc, clear all, close all;

% Define waypoints along the x-axis
waypoints = [0; 2; 0; 5];

% Total time for the trajectory
total_time = 2;  % Example total time

% Number of waypoints
num_waypoints = size(waypoints, 1);

% Calculate distances between consecutive waypoints
distances = zeros(num_waypoints - 1, 1);
for i = 1:(num_waypoints - 1)
    distances(i) = norm(waypoints(i + 1, :) - waypoints(i, :));
end

% Total distance
total_distance = sum(distances);

% Allocate time proportionally to each segment
time_segments = (distances / total_distance) * total_time

% Generate the cumulative time vector
time = [0; cumsum(time_segments)];

% Number of segments
N = length(waypoints) - 1;

% Normalize time variables
t_init = 0
t_med1 = time_segments(1) / total_time
t_med2 = (time_segments(1) + time_segments(2)) / total_time
t_final = (time_segments(1) + time_segments(2) + time_segments(3))/total_time

%% Number of coefficients
numCoeff = 10;

%% Initial Time Constraints (Normalized)
A_init = [position_time(t_init)', zeros(1, 10), zeros(1, 10);...
          position_time(t_med1)', zeros(1,10), zeros(1, 10);...
          velocity_time(t_init)', zeros(1, 10), zeros(1, 10);...
          acceleration_time(t_init)', zeros(1, 10), zeros(1, 10)];

b_init = [waypoints(1); waypoints(2); 0; 0];

%% Constraints for Midpoint 1 (velocity, acceleration, jerk - Normalized)
A_v_m1 = [velocity_time(t_med1)', zeros(1, 10), zeros(1, 10);...
          zeros(1, 10), -velocity_time(t_med1)', zeros(1, 10)];

b_v_m1 = [0; -0];

A_a_m1 = [acceleration_time(t_med1)', zeros(1, 10), zeros(1, 10);...
          zeros(1, 10), -acceleration_time(t_med1)', zeros(1, 10)];

b_a_m1 = [0; 0];

A_j_m1 = [jerk_time(t_med1)', zeros(1, 10), zeros(1, 10);...
          zeros(1, 10), -jerk_time(t_med1)', zeros(1, 10)];

b_j_m1 = [0; 0];

%% Constraints for Midpoint 2 (position, velocity, acceleration, jerk - Normalized)
A_x_m2 = [zeros(1, 10), position_time(t_med1)', zeros(1, 10);...
          zeros(1, 10), position_time(t_med2)', zeros(1,10)];

b_x_m2 = [waypoints(2); waypoints(3)];

A_v_m2 = [zeros(1, 10), velocity_time(t_med2)', zeros(1, 10);...
          zeros(1, 10), zeros(1, 10), -velocity_time(t_med2)'];

b_v_m2 = [-0; 0];

A_a_m2 = [zeros(1, 10), acceleration_time(t_med2)', zeros(1, 10);...
          zeros(1, 10), zeros(1, 10), -acceleration_time(t_med2)'];

b_a_m2 = [0; 0];

A_j_m2 = [zeros(1, 10), jerk_time(t_med2)', zeros(1, 10);...
          zeros(1, 10), zeros(1, 10), -jerk_time(t_med2)'];

b_j_m2 = [0; 0];

%% Final Time Constraints (Normalized)
A_final = [zeros(1, 10), zeros(1, 10), position_time(t_med2)';...
           zeros(1, 10), zeros(1, 10), position_time(t_final)';...
           zeros(1, 10), zeros(1, 10), velocity_time(t_final)';...
           zeros(1, 10), zeros(1, 10), acceleration_time(t_final)'];

b_final = [waypoints(3); waypoints(4); 0; 0];

%% Combine all constraints
A = [A_init; A_v_m1; A_a_m1; A_j_m1; A_x_m2; A_v_m2; A_a_m2; A_j_m2; A_final];
b = [b_init; b_v_m1; b_a_m1; b_j_m1; b_x_m2; b_v_m2; b_a_m2; b_j_m2; b_final];

%% Hessian matrix for the normalized time
H_m1 = hessian_cost(t_med1);
H_i = hessian_cost(t_init);
H_m2 = hessian_cost(t_med2);
H_f = hessian_cost(t_final);

H_1 = H_m1 - H_i;
H_2 = H_m2 - H_m1;
H_3 = H_f - H_m2;

H = [H_1, zeros(10, 10), zeros(10, 10);...
     zeros(10, 10), H_2, zeros(10, 10);...
     zeros(10, 10), zeros(10, 10), H_3];

%% Solve the optimization problem
coeff_x = quadprog(H, [], [], [], A, b);
% 
% %% Plot the trajectory
% figure;
% hold on;
% plot([t_init, t_med1, t_med2, t_final], waypoints, 'ro--', 'LineWidth', 2);
% for i = 1:N
%     t = linspace(time(i), time(i + 1), 100);
%     idx = (i - 1) * numCoeff + 1:i * numCoeff;
%     px = coeff_x(idx)' * position_time((t - time(i)) / (time(i + 1) - time(i)));
%     plot(t, px, 'b-', 'LineWidth', 1.5);
% end
% xlabel('Normalized Time');
% ylabel('px');
% grid on;
% title('Normalized Minimum Snap Trajectory between Two Waypoints');
% hold off;
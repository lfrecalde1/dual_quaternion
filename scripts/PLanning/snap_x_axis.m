

% Define waypoints along the x-axis
waypoints = [0; 10];

% Time allocation for the two waypoints
time = [0, 1];

% Degree of the polynomial (7th degree for minimum snap)
deg = 7;

% Total number of coefficients per segment
numCoeff = deg + 1;

% Construct the constraint matrices for the two waypoints
t0 = time(1);
t1 = time(2);

% Initial and final position constraints
A = [polyT(t0, deg); polyT(t1, deg)];
b = [waypoints(1); waypoints(2)];

% Build cost matrix H for minimum snap (integral of snap squared)
H = polyH(t0, t1, deg)

% Ensure H is positive definite for quadprog


% Solve the optimization problem
coeff_x = quadprog(H, [], [], [], A, b)

% Plot the trajectory
figure;
hold on;
plot(time, waypoints, 'ro--', 'LineWidth', 2);
t = linspace(t0, t1, 100);
x = polyval(flip(coeff_x'), t);
plot(t, x, 'b-', 'LineWidth', 1.5);
xlabel('Time');
ylabel('X');
grid on;
title('Minimum Snap Trajectory between Two Waypoints');
hold off;


% Function to generate the polynomial matrix
function T = polyT(t, deg)
    T = zeros(1, deg + 1);
    for i = 0:deg
        T(i + 1) = t^i;
    end
end

% Function to generate the cost matrix for minimum snap
function H = polyH(t0, t1, deg)
    H = zeros(deg + 1);
    for i = 4:deg
        for j = 4:deg
            H(i + 1, j + 1) = (i * (i - 1) * (i - 2) * (i - 3) * j * (j - 1) * (j - 2) * (j - 3) / (i + j - 7)) * (t1^(i + j - 7) - t0^(i + j - 7));
        end
    end
end


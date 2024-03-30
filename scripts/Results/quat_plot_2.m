clc, clear all,close all;
% Define the quaternion (w + xi + yj + zk)
% Example: quaternion for 90 degrees rotation around the z-axis
q = [cos(pi/4), 0, 0, sin(pi/4)]; % [w, x, y, z]

% Convert quaternion to rotation matrix
% If you have the Aerospace Toolbox:
R = quat2rotm(q);
% Otherwise, manually implement the conversion if needed

% Define the original coordinate axes (could also use more complex objects)
points = eye(3); % Simple identity matrix for demonstration (3 axes)

% Apply the rotation
rotatedPoints = R * points;

% Plotting
figure;
hold on;
axis equal;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Quaternion Rotation Visualization');

% Original axes
quiver3(0, 0, 0, points(1,1), points(2,1), points(3,1), 'r', 'LineWidth', 2);
quiver3(0, 0, 0, points(1,2), points(2,2), points(3,2), 'g', 'LineWidth', 2);
quiver3(0, 0, 0, points(1,3), points(2,3), points(3,3), 'b', 'LineWidth', 2);

% Rotated axes
quiver3(0, 0, 0, rotatedPoints(1,1), rotatedPoints(2,1), rotatedPoints(3,1), 'r:', 'LineWidth', 2);
quiver3(0, 0, 0, rotatedPoints(1,2), rotatedPoints(2,2), rotatedPoints(3,2), 'g:', 'LineWidth', 2);
quiver3(0, 0, 0, rotatedPoints(1,3), rotatedPoints(2,3), rotatedPoints(3,3), 'b:', 'LineWidth', 2);

legend('X axis', 'Y axis', 'Z axis', 'Rotated X axis', 'Rotated Y axis', 'Rotated Z axis');
hold off;
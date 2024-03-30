clc, clear all, close all;
% Example Quaternion (q = [w, xi, yj, zk])
% For demonstration, we're using a simple quaternion. Replace it with your quaternion.
q = [0.7071, 0.7071, 0, 0]; % This is a quaternion representing a rotation

% Extract the vector part (xi, yj, zk)
qv = q(2:4);

% Normalize the vector part to lie on the sphere's surface
qv_norm = qv / norm(qv);

% Plot the unit sphere
figure;
[x, y, z] = sphere;
surface = surf(x, y, z); % Generate a unit sphere
surface.FaceColor = 'cyan';
surface.FaceAlpha = 0.5;
surface.EdgeColor = 'none';
hold on;

% Adjust the appearance
axis equal; % Equal aspect ratio for all axes
xlabel('x');
ylabel('y');
zlabel('z');
title('Quaternion Vector on Sphere');

% Plot the quaternion's vector part
% We use quiver3 to plot the vector starting from the origin
quiver3(0, 0, 0, qv_norm(1), qv_norm(2), qv_norm(3), 'r', 'LineWidth', 2);
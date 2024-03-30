% Define the quaternion
% Replace with your quaternion values
q = [1, 0, 0, 0]; % Identity quaternion for no rotation

% Convert quaternion to rotation matrix
% If you have the Aerospace Toolbox:
% R = quat2rotm(q);
% Otherwise, use your custom conversion function
R = quat2rotm_manual(q);

% Define a point on the unit sphere
x = [1; 0; 0];

% Create a tangent plane at point x
[U,~,V] = svd([x null(x')]);

% Plane size
plane_size = 0.5;
[u,v] = meshgrid(linspace(-plane_size, plane_size, 10));

% Flatten the grid to 2D
u = u(:);
v = v(:);

% Points on the plane
P = U(:,2:3)*[u';v'];

% Add the original point to get coordinates on the tangent plane
P = x + P;

% Plot the unit sphere
figure;
[X, Y, Z] = sphere;
h = surf(X, Y, Z, 'FaceColor', 'none', 'EdgeColor', [0.5 0.5 0.5]);
alpha(h, 0.5);
hold on;

% Plot point x
plot3(x(1), x(2), x(3), 'ko', 'MarkerSize', 10, 'LineWidth', 2);

% Plot tangent plane
P = reshape(P, 3, [], size(P,2));
h_plane = patch(P(1,:), P(2,:), P(3,:), 'g');
alpha(h_plane, 0.3);

% Define a vector v on the tangent plane
v = V(:,2) * 0.5;

% Plot vector v
quiver3(x(1), x(2), x(3), v(1), v(2), v(3), 'k', 'LineWidth', 2);

% Apply rotation to vector v
v_rot = R * v;

% Plot the rotated vector
quiver3(x(1), x(2), x(3), v_rot(1), v_rot(2), v_rot(3), 'r', 'LineWidth', 2);

% Adjust the view
axis equal;
axis vis3d;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Quaternion Rotation on Sphere with Tangent Plane');
grid on;
view(35, 45);
hold off;

function R = quat2rotm_manual(q)
    % Your implementation to convert quaternion to rotation matrix
    % ...
end
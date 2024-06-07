function [R] = QuatToRot(quat)
%Quaternion to Rotational Matrix
%   Detailed explanation goes here
%% Definition of the quaternion
q = quat;
q = q/norm(q, 2);

%% Matrix conformed of the quaternions
q_hat = zeros(3,3);

q_hat(1, 2) = -q(4);
q_hat(1, 3) = q(3);
q_hat(2, 3) = -q(2);
q_hat(2, 1) = q(4);
q_hat(3, 1) = -q(3);
q_hat(3, 2) = q(2);

R = eye(3) + 2*q_hat*q_hat + 2*q(1)*q_hat;
end
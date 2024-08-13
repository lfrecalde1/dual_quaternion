function [H] = H_matrix(t_init, t_final)
%UNTITLED2 Summary of this function D = blkdiag(A, A, A);goes here
%   Detailed explanation goes here
%% Hessian
H_f = hessian_cost(t_final);
H_i = hessian_cost(t_init);

Hx = H_f - H_i;
% Solve the optimization problem
H = blkdiag(Hx, Hx, Hx);
end
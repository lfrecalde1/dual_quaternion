clc, clear all, close all

% CasADi v3.4.
addpath('/home/fer/casadi-linux-matlabR2014b-v3.4.5');
import casadi.*;
%% Program to code Quadratic programming ofr Trajectory planning
degree = 7;
coeff = degree + 1;
derivative_number = 4;
p_c = SX.sym('p_c', coeff);
t_c = SX.sym('t_c', 1);

%% Polynomila vector time
T = SX.ones(degree + 1, 1);
for i = 1:degree
    T(i + 1, 1) = t_c^i;
end

%% Polynomial function
P_x = p_c'*T;

%% Derivative time
D_P_x = P_x;
for i=1:derivative_number
    D_P_x = gradient(D_P_x, t_c);
end
vector_t = jacobian(D_P_x, p_c)

%% Polynomial time r derivative 
dr_P_x = p_c'*vector_t';

%% Compute snap square
snap_square = dr_P_x*dr_P_x;
hessian_snap = hessian(snap_square, p_c);

func_hessian = Function('func_hessian', {t_c}, {hessian_snap});

%% Compute hessian
% Time allocation for the two waypoints
time = [0.0, 2.0];

t0 = time(1);
t1 = time(2);

H_0 = full(func_hessian(t0));
H_F = full(func_hessian(t1))


% %% Function Gnerate with Casai
% func_poly_t = Function('func_poly_t', {t_c},{T});
% func_poly_dt = Function('func_polu_dt', {t_c}, {vector_t});
% %% Check function functions
% value_vector =  full(func_poly_t(0.5))
% value_vector_aux =  full(func_poly_dt(0.5))

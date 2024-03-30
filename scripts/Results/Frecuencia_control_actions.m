%% Control actions different behavior
%% Results DQ and Classical Formulation
clc, clear all, close all;

%% Load Matrices
separed = load("Separed_cost_without_velocities_0.5.mat");
dual = load("Dual_cost_without_velocities_0.5.mat");

% Control Values
F_dual = dual.F;
F_separed = separed.F;

M_dual = dual.M;
M_separed = separed.M;


t_dual = dual.t;
t_separed = separed.t;

t_dual = t_dual(1, 1:length(F_dual));
t_separed = t_separed(1, 1:length(F_separed));


% States of the system
X_dual = dual.x;
X_separed = separed.x;

Aux_X_dual = zeros(30, 7, length(X_dual));
Aux_X_separed = zeros(30, 7, length(X_separed));

for i=1:size(Aux_X_dual, 1)
    for j=1:size(Aux_X_dual, 3)
        position = X_separed(i, 1:3, j);
        quaternion = X_separed(i, 7:10, j);
        Aux_X_separed(i, :, j) = [position, quaternion]';
    end
    
end

for i=1:size(Aux_X_dual, 1)
    for j=1:size(Aux_X_dual, 3)
        quaternion = X_dual(i, 1:4, j)';
        position = get_traslatation_dual(X_dual(i, 1:8, j));
        Aux_X_dual(i, :, j) = [position(2:4); quaternion];
    end
    
end
cost_t_dual = dual.translation_cost;
cost_t_separed = separed.translation_cost;

cost_q_dual = dual.orientation_cost;
cost_q_separed = separed.orientation_cost;

Aux_cost_dual = zeros(30, 3, length(cost_t_dual));
Aux_cost_separed = zeros(30, 3, length(cost_t_dual));

for i=1:size(cost_t_dual, 1)
    for j=1:size(cost_t_dual, 3)
        Aux_cost_dual(i, 1, j) = cost_t_dual(i, 1, j);
        Aux_cost_dual(i, 2, j) = cost_q_dual(i, 1, j);
        Aux_cost_dual(i, 3, j) = cost_t_dual(i, 1, j) +  cost_q_dual(i, 1, j);
        
        Aux_cost_separed(i, 1, j) = cost_t_separed(i, 1, j);
        Aux_cost_separed(i, 2, j) = cost_q_separed(i, 1, j);
        Aux_cost_separed(i, 3, j) = cost_t_separed(i, 1, j) +  cost_q_separed(i, 1, j);
    end
    
end

Fs = 1/(t_dual(2)-t_dual(1));
L = length(F_separed);
Experiment_separed = reshape(M_separed(20, 1, :), 1, length(F_separed));
Experiment_dual = reshape(M_dual(20, 1, :), 1, length(F_separed));

Fourier_separed = fft(Experiment_separed);
Fourier_dual = fft(Experiment_dual);


P2_separed = abs(Fourier_separed/L);
P1_separed = P2_separed(1:L/2+1);
P1_separed(2:end-1) = 2*P1_separed(2:end-1);

P2_dual = abs(Fourier_dual/L);
P1_dual = P2_dual(1:L/2+1);
P1_dual(2:end-1) = 2*P1_dual(2:end-1);

figure
f = Fs/L*(0:(L/2));
plot(f,P1_separed,"LineWidth",3) 
hold on;
plot(f,P1_dual,"LineWidth",3) 
title("Single-Sided Amplitude Spectrum of X(t)")
xlabel("f (Hz)")
ylabel("|P1(f)|")


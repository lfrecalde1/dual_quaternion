function [ISE_dual_plot, Ise_dual] = ISE_t(Data_dual, t_dual)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
ISE_dual_plot = [];
Ise_dual = [];
dt = t_dual(2)-t_dual(1);
number_experiments = size(Data_dual, 2);
for i =1:number_experiments
    Ise_dual_aux = 0;
    for k=1:length(t_dual)
        Ise_dual_aux = Ise_dual_aux + (t_dual(k))*(sqrt(Data_dual(k, i)))*(dt);
        ISE_dual_plot(i, k) = Ise_dual_aux;   
    end
    ISE_dual_plot(i, :) = ISE_dual_plot(i, :)/1;
    Ise_dual(1, i) =  Ise_dual_aux;
end
Ise_dual = Ise_dual/1;
ISE_dual_plot = mean(ISE_dual_plot);
end


function [h] = trajectory_time(t_init_trajectory)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
%% Initial States second second
zi = 2;
w_d = 2;
x_init = zi*cos(w_d*t_init_trajectory);
x_init_d = -zi*w_d*sin(w_d*t_init_trajectory);
x_init_dd = -zi*w_d*w_d*cos(w_d*t_init_trajectory);
x_init_ddd = zi*w_d*w_d*w_d*sin(w_d*t_init_trajectory);
x_init_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_init_trajectory);

h = [x_init; x_init_d; x_init_dd; x_init_ddd; x_init_dddd];
end
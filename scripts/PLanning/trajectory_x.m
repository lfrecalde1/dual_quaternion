function [h, h_d, h_dd, h_ddd, h_dddd] = trajectory_x(zi, w_d, t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Trajectory Points x
x = zi*cos(w_d*t);
x_d = -zi*w_d*sin(w_d*t);
x_dd = -zi*w_d*w_d*cos(w_d*t);
x_ddd = zi*w_d*w_d*w_d*sin(w_d*t);
x_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t);


%% Complete Data
h = x;
h_d = x_d;
h_dd = x_dd;
h_ddd = x_ddd;
h_dddd = x_dddd;

end
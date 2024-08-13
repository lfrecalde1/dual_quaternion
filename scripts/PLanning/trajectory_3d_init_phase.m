function [h, h_d, h_dd, h_ddd, h_dddd] = trajectory_3d_init_phase(coeff, t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
h = [];
h_d = [];
h_dd = [];
h_ddd = [];
h_dddd = [];
for k=1:size(coeff, 2)
    p = coeff(:, k)' * position_time(t);
    v = coeff(:, k)' * velocity_time(t);
    a = coeff(:, k)' * acceleration_time(t);
    j = coeff(:, k)' * jerk_time(t);
    s = coeff(:, k)' * snap_time(t);
    h = [h;p];
    h_d = [h_d;v];
    h_dd = [h_dd; a];
    h_ddd = [h_ddd; j];
    h_dddd = [h_dddd; s];
end
end
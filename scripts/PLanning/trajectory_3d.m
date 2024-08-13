function [h, h_d, h_dd, h_ddd, h_dddd] = trajectory_3d(zi, w_d, t)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Trajectory Points x
x = zi*cos(w_d*t);
x_d = -zi*w_d*sin(w_d*t);
x_dd = -zi*w_d*w_d*cos(w_d*t);
x_ddd = zi*w_d*w_d*w_d*sin(w_d*t);
x_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t);

%% Trajectory Points y
y = zi*sin(w_d*t);
y_d = zi*w_d*cos(w_d*t);
y_dd = -zi*w_d*w_d*sin(w_d*t);
y_ddd = -zi*w_d*w_d*w_d*cos(w_d*t);
y_dddd = zi*w_d*w_d*w_d*w_d*sin(w_d*t);

%% Trajectory Point z
z = 1 + 0.1*t;
z_d = 0.1*ones(1, size(t, 2));
z_dd = 0*ones(1, size(t, 2));
z_ddd = 0*ones(1, size(t, 2));
z_dddd = 0*ones(1, size(t, 2));

%% Complete Data
h = [x;y;z];
h_d = [x_d; y_d; z_d];
h_dd = [x_dd; y_dd; z_dd];
h_ddd = [x_ddd; y_ddd; z_ddd];
h_dddd = [x_dddd; y_dddd; z_dddd];

end
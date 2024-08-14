function [h_init] = trajectory_3d_init_final(zi, w_d, x_init, v_init, a_init, j_init, s_init, t_init_trajectory)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% initial Points x
x = zi*cos(w_d*t_init_trajectory);
x_d = -zi*w_d*sin(w_d*t_init_trajectory);
x_dd = -zi*w_d*w_d*cos(w_d*t_init_trajectory);
x_ddd = zi*w_d*w_d*w_d*sin(w_d*t_init_trajectory);
x_dddd = zi*w_d*w_d*w_d*w_d*cos(w_d*t_init_trajectory);



%% Initial Points y
y = zi*sin(w_d*t_init_trajectory);
y_d = zi*w_d*cos(w_d*t_init_trajectory);
y_dd = -zi*w_d*w_d*sin(w_d*t_init_trajectory);
y_ddd = -zi*w_d*w_d*w_d*cos(w_d*t_init_trajectory);
y_dddd = zi*w_d*w_d*w_d*w_d*sin(w_d*t_init_trajectory);


%% Initial Point z
z = 1 + 0.1*t_init_trajectory;
z_d = 0.1;
z_dd = 0;
z_ddd = 0;
z_dddd = 0;

%% Aux points
h_init = [x_init(1); v_init(1); a_init(1); j_init(1); s_init(1); x; x_d; x_dd; x_ddd; x_dddd; x_init(2); v_init(2); a_init(2); j_init(2); s_init(2); y; y_d; y_dd; y_ddd; y_dddd; x_init(3); v_init(3); a_init(3); j_init(3); s_init(3); z; z_d; z_dd; z_ddd; z_dddd];
end
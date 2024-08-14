function [h_init] = trajectory_3d_init_final_aux(zi, w_d, x_init, v_init, a_init, j_init, s_init, t_init_trajectory)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% initial Points x
x = 0;
x_d = 0;
x_dd = 0;
x_ddd = 0;
x_dddd = 0;



%% Initial Points y
y = 0;
y_d = 0;
y_dd = 0;
y_ddd = 0;
y_dddd = 0;


%% Initial Point z
z = 0;
z_d = 0;
z_dd = 0;
z_ddd = 0;
z_dddd = 0;

%% Aux points
h_init = [x_init(1); v_init(1); a_init(1); j_init(1); s_init(1); x; x_d; x_dd; x_ddd; x_dddd; x_init(2); v_init(2); a_init(2); j_init(2); s_init(2); y; y_d; y_dd; y_ddd; y_dddd; x_init(3); v_init(3); a_init(3); j_init(3); s_init(3); z; z_d; z_dd; z_ddd; z_dddd];
end
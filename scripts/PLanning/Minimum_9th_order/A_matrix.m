function [A] = A_matrix(t_final)
%UNTITLED2 Summary of this function D = blkdiag(A, A, A);goes here
%   Detailed explanation goes here
A = [position_time(t_final)';...
     velocity_time(t_final)';...
     acceleration_time(t_final)';...
     jerk_time(t_final)';...
     snap_time(t_final)'];
end
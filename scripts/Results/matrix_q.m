function [Q1_aux] = matrix_q(q1)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
Q1_aux = [q1(1), -q1(2:4)';...
          q1(2:4), eye(3)*q1(1) + hat_map(q1(2:4))];
end


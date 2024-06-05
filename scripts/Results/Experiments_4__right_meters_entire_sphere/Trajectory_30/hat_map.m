function [S] = hat_map(q)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here
S = [0, -q(3), q(2);...
     q(3), 0, -q(1);...
     -q(2), q(1), 0];
end


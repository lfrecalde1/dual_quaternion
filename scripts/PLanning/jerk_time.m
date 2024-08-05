function [vector] = jerk_time(t)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
vector = [0*ones(1, size(t, 2)); 0*ones(1, size(t, 2)); 0*ones(1, size(t, 2)); 6*ones(1, size(t, 2)); 24*t; 60*t.^2; 120*t.^3; 210*t.^4];
end
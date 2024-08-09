function [vector] = snap_time(t)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
vector = [0*ones(1, size(t, 2)); 0*ones(1, size(t, 2)); 0*ones(1, size(t, 2)); 0*ones(1, size(t, 2)); 24*ones(1, size(t, 2)); 120*t; 360*t.^2; 840*t.^3; 1680*t.^4; 3024*t.^5];
end
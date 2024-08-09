function [vector] = velocity_time(t)
%UNTITLED11 Summary of this function goes here
%   Detailed explanation goes here
vector = [0*ones(1, size(t, 2)); 1*ones(1, size(t, 2)); 2*t; 3*t.^2; 4*t.^3; 5*t.^4; 6*t.^5; 7*t.^6; 8*t.^7; 9*t.^8];
end
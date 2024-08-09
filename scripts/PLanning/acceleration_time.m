function [vector] = acceleration_time(t)
%UNTITLED12 Summary of this function goes here
%   Detailed explanation goes here
vector = [0*ones(1, size(t, 2)); 0*ones(1, size(t, 2)); 2*ones(1, size(t, 2)); 6*t; 12*t.^2; 20*t.^3; 30*t.^4; 42*t.^5; 56*t.^6; 72*t.^7];
end
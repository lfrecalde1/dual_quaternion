function [vector] = position_time(t)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
vector = [0*ones(1, size(t, 2)) ;t; t.^2; t.^3; t.^4; t.^5; t.^6; t.^7];
end
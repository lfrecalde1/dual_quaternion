function [vector] = position_time(t)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
vector = [1*ones(1, size(t, 2)) ;t; t.^2; t.^3; t.^4; t.^5; t.^6; t.^7; t.^8; t.^9];
end
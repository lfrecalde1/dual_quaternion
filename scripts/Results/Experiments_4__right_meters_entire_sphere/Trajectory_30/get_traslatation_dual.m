function [t] = get_traslatation_dual(h1)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
h1p = h1(1:4);
h1d = h1(5:8);
h1d = [h1d(1), h1d(2), h1d(3), h1d(4)]';
h1p_c = [h1p(1), -h1p(2), -h1p(3), -h1p(4)]';
t = 2 * quaternion_multiply(h1d, h1p_c);
end


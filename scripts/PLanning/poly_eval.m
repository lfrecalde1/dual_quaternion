function [f] = poly_eval(P, t)
%UNTITLED9 Summary of this function goes here
%   Detailed explanation goes here
p_0 = P(1);
p_1 = P(2);
p_2 = P(3);
p_3 = P(4);
p_4 = P(5);
p_5 = P(6);
p_6 = P(7);
p_7 = P(8);
f = p_7*t^7 + p_6*t^6 + p_5*t^5 + p_4*t^4 + p_3*t^3 + p_2*t^2 + p_1*t + 1*p_0;
end
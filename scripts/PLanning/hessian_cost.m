function [H] = hessian_cost(t)
%UNTITLED14 Summary of this function goes here
%   Detailed explanation goes here
H =  [[0, 0, 0, 0, 0,          0,           0,           0,            0,            0]
[0, 0, 0, 0, 0,          0,           0,           0,            0,            0]
[0, 0, 0, 0, 0,          0,           0,           0,            0,            0]
[0, 0, 0, 0, 0,          0,           0,           0,            0,            0]
[0, 0, 0, 0, 0,          0,           0,           0,            0,            0]
[0, 0, 0, 0, 0,    14400*t,   43200*t^2,  100800*t^3,   201600*t^4,   362880*t^5]
[0, 0, 0, 0, 0,  43200*t^2,  172800*t^3,  453600*t^4,   967680*t^5,  1814400*t^6]
[0, 0, 0, 0, 0, 100800*t^3,  453600*t^4, 1270080*t^5,  2822400*t^6,  5443200*t^7]
[0, 0, 0, 0, 0, 201600*t^4,  967680*t^5, 2822400*t^6,  6451200*t^7, 12700800*t^8]
[0, 0, 0, 0, 0, 362880*t^5, 1814400*t^6, 5443200*t^7, 12700800*t^8, 25401600*t^9]];
end
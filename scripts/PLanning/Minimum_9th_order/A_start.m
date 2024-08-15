function [A] = A_start()
%UNTITLED2 Summary of this function D = blkdiag(A, A, A);goes here
%   Detailed explanation goes here
%construct equility constraints
A = [1 0 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0 0;
     0 0 2 0 0 0 0 0 0 0;
     0 0 0 6 0 0 0 0 0 0;
     0 0 0 0 24 0 0 0 0 0];

end
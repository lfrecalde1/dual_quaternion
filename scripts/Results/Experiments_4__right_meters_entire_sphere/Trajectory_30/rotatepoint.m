function [p_projected] = rotatepoint(q, p)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
p_q = [0; p(1); p(2); p(3)];

q_c = [q(1); -q(2); -q(3); -q(4)];

q_aux = quaternion_multiply(q_c, p_q);
q_final = quaternion_multiply(q_aux', q');

p_projected = q_final(2:4);
end


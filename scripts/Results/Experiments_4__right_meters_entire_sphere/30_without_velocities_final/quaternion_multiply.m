function [output_aux] = quaternion_multiply(q1, q2)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
a1 = q1(1);
b1 = q1(2);
c1 = q1(3);
d1 = q1(4);

a2 = q2(1);
b2 = q2(2);
c2 = q2(3);
d2 = q2(4);


aux_1 = a1*a2 - b1*b2 -c1*c2 - d1*d2;
aux_2 = a1*b2 +b1*a2 + c1*d2 - d1*c2;
aux_3 = a1*c2 - b1*d2 + c1*a2 + d1*b2;
aux_4 = a1*d2 + b1*c2 - c1*b2 + d1*a2;

output = [aux_1; aux_2; aux_3; aux_4];

Q1 = [a1, -b1, -c1, -d1;...
      b1, a1, -d1, c1;...
      c1, d1, a1, -b1;...
      d1, -c1, b1, a1];
  
Q2 = [a2, -b2, -c2, -d2;...
      b2, a2, -d2, c2;...
      c2, d2, a2, -b2;...
      d2, -c2, b2, a2];

%% Better formulation

Q_aux = Q1*q2;

output_aux= Q_aux;
end


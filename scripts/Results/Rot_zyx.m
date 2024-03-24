function [R] = Rot_zyx(euler)
%UNTITLED2 Summary of this function goes here

%% Euler values
phi = euler(1);
theta =euler(2);
psi = euler(3);

%% Matrix X
RotX = [1 0 0;...
        0 cos(phi) -sin(phi);...
        0 sin(phi) cos(phi)];
    
RotY = [cos(theta) 0 sin(theta);...
        0 1 0;...
        -sin(theta) 0 cos(theta)];
    
RotZ = [cos(psi) -sin(psi) 0;...
        sin(psi) cos(psi) 0;...
        0 0 1];

R = RotZ*RotY*RotX;
   
end
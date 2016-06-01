function [ J ] = angularRateMtx( angles )
% angularRateMtx Generates the jacobian that transform a robot angular
% velocity measured in the world frame to the corresponding velocity on the
% robot frame
% INPUTS:
% Angles phi, theta and psi in radians
%   phi   :   rotation around X-axis
%   theta :   rotation around Y-axis
%   psi   :   rotation around Z-axis
% OUTPUT:
%   J     :   angular rate matrix
% Reference: Section 43.2.1, Handbook of Robotics (Springer) 

phi = angles(1);
theta = angles(2);
J = [   1   0   -sin(theta) ;
        0   cos(theta)  cos(theta)*sin(phi) ;
        0   -sin(phi)   cos(theta)*cos(phi) ;];
end
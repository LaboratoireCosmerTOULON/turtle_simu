function [ M ] = expMapDirectRxRyRz( v, dt )
% Computes the transformation from the pose t to the pose t+dt
% Input : the velocity (m/s,rad/s) and the time to apply it (s) 

vThetaU = vThetaUFromVRxRyRz(v);
M = expMapDirectThetaU(vThetaU,dt);
 
end


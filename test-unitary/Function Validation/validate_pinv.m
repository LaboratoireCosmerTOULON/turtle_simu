close all;
clear; 
clc;  

addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/visualServoing

% Rope parameter
rlen = 0.70; % cable half-length in meters
hmax = 0.40; % cable maximum sag
% Turtlebot features (measurements in SigmaC)
Tx      = -0.01;          % distance in x-axis between camera and rope attachment on robot
Ty      = -0.11;        % distance in y-axis between camera and rope attachment on robot
Tz      =  0.24;         % distance in z-axis between camera and rope attachment on robot
Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame

% 1: s_gna= (0.759295,-0.861092)


a = 0.748344;
b = -0.850352;
s = [a; b];
Pcat3d_gna = catenary3D(rlen,hmax,s,Tcam,100);
p_A = Pcat3d_gna(:,end); % coordinates of rope attachment point at robot r1 in rope-frame (p_C)
L = interactionMatrix(rlen,hmax,s, p_A(1), p_A(2), p_A(3));

LP2 = L'*inv(L*L');
LP2
Lpinv = pinv(L);
Lpinv

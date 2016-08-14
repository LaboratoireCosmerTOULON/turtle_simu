% SCRIPT
%   Validate catenary 3D equation

% Close all opened figures and clear workspace
close all;
clear; 
clc;
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models

rlen = 0.70;
hmax = 0.40;
a = 0.5;
b = 0.5;
s = [a; b];

% Turtlebot features (measurements in SigmaC)
Tx      = -0.01;          % distance in x-axis between camera and rope attachment on robot
Ty      = -0.11;        % distance in y-axis between camera and rope attachment on robot
Tz      = 0.24;         % distance in z-axis between camera and rope attachment on robot
Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame

% Catenary 3D equation in camera frame
Pcat3d = catenary3D(rlen,hmax,s,Tcam,1000);

h 	= a*hmax;

C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);
t = D;
xA = -b*(t+D) + Tx
yA = -(1/C)*(cosh(C*t)-1) + h + Ty
zA = sqrt(1-b^2)*(t+D) + Tz

Pcat3d(:,end)

% figure();
% plot3(Pcat3d(1,:),Pcat3d(2,:),Pcat3d(3,:));
% title('Catenary')
% xlabel('X(m)')
% ylabel('Y(m)')
% zlabel('Z(m)')




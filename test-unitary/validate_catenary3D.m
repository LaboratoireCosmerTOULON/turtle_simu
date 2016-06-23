% SCRIPT
%   Validate catenary 3D equation

% Close all opened figures and clear workspace
close all;
clear; 
clc;
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models


% Rope parameter
rlen    = 0.75;     % cable half-length in meters
hmax    = 0.9*rlen; % cable maximum sag

% Catenary parameters
% a = h/hmax
% b = sin(theta), where theta is the rope angle with the robot longitudinal
% axis
a       = 0.6; 
h       = a*hmax;       % calbe current sag
theta   = deg2rad(0);  % cable angle in radians
b       = sin(theta);
s       = [a; b];

% Constants for the catenary equation
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);

% Turtlebot features (measurements in SigmaC)
Tx      = 0.0;          % distance in x-axis between camera and rope attachment on robot
Ty      = -0.11;        % distance in y-axis between camera and rope attachment on robot
Tz      = 0.28;         % distance in z-axis between camera and rope attachment on robot
Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame
f   = 1;      % focal length

% Catenary 3D equation in camera frame
Pcat3d = catenary3D(rlen,hmax,s,Tcam);

% Rotate axis for projection
rotm = eul2rotm([0 0 -pi/2]);
Pcat3d_plt = Pcat3d;
for i=1:length(Pcat3d)
    Pcat3d_plt(:,i) = rotm*Pcat3d(:,i);
end

figure();
plot3(Pcat3d_plt(1,:),Pcat3d_plt(2,:),Pcat3d_plt(3,:)) % projection classique
% l=legend('Projection p = f*P/Z','Projection Vincent)','Projection Matheus)');l.Location='best'; %'Gauss-Newton','Validation'
title('Catenary')
xlabel('X(m)')
ylabel('Y(m)')
zlabel('Z (m)')




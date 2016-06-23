% SCRIPT

% Close all opened figures and clear workspace
close all;
clear; 
clc;

addpath /home/matheus/Documents/MATLAB/turtle_simu/src/data
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/drawing
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/estimation
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/optimization
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/sampling

% Rope parameter
rlen    = 0.75;     % cable half-length in meters
hmax    = 0.9*rlen; % cable maximum sag
% Catenary parameters
% a = h/hmax
% b = sin(theta), where theta is the rope angle with the robot longitudinal
% axis
a       = 0.6; 
h       = a*hmax;       % calbe current sag
theta   = deg2rad(30);  % cable angle in radians
b       = sin(theta);

% Constants for the catenary equation
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);

% Rope equation in frame Sigma1
sigma1_x        = linspace(-D,D,100);
sigma1_y        = zeros(1, length(sigma1_x));
sigma1_z        = 1/C*(cosh(C*sigma1_x) - 1);
sigma1_Pcat     = [sigma1_x; sigma1_y; sigma1_z; ones(1,length(sigma1_x))];

% Frame changing : Sigma1 --> Sigma2 
sigma2_R_sigma1 = eul2rotm([ theta 0 0]);
sigma2_T_sigma1 = [D*cos(theta); D*sin(theta); -h];
sigma2_M_sigma1 = [sigma2_R_sigma1, sigma2_T_sigma1;
                    0 0 0 1 ];

sigma2_Pcat = zeros(4,length(sigma1_Pcat(1,:)));
for i=1:length(sigma1_Pcat)
    sigma2_Pcat(:,i) = sigma2_M_sigma1*sigma1_Pcat(:,i);
end

figure();
plot3(sigma1_Pcat(1,:),sigma1_Pcat(2,:),sigma1_Pcat(3,:)) % original curve
title('Catenary Sigma1')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')
figure();
plot3(sigma2_Pcat(1,:),sigma2_Pcat(2,:),sigma2_Pcat(3,:)) % original curve
title('Catenary Sigma2')
xlabel('X (m)')
ylabel('Y (m)')
zlabel('Z (m)')

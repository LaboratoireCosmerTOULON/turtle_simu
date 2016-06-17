% SCRIPT
%   Plot a catenary 3D curve and its projection on the image plane 
%   Estimate the catenary parameters from its projection
%   Unities : centimeters and radians

% Close all opened figures and clear workspace
close all;
clear; 
clc;    

addpath /home/matheus/Documents/MATLAB/turtle_simu/src/data
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/estimation
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/optimization
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/sampling

% Read measur
fileID = fopen('/home/matheus/Documents/MATLAB/turtle_simu/src/data/rope2d_160615.dat','r');
sizeA = [2 Inf];
data   = fscanf(fileID,'%f %f', sizeA);
xi  = -data(1,:) -0.06667;
yi  = -data(2,:) -0.4533;
fclose(fileID);

% Rope parameter
rlen = 0.72; % cable half-length in meters
hmax = 0.9*rlen; % cable maximum sag

% Fitting with Gauss-Newton method
pinit = [0.5; 0.5]; % set initial guess
lb = [0.01; 0.01]; % lower bound
ub = [1; 1]; % upper bound
[p_gna,steps,chisq] = GaussNewton_v2(xi,yi,rlen,hmax,pinit,lb,ub); % minimization with dampled GNA
% Fitting with FMINCON
A = []; % -1 0; 1 0; 0 -1; 0 1
b = []; % 0; 50; 0; 1
Aeq = [];
beq = [];
p_fmc = fmincon(@(p_hat)fcout1(xi,yi,rlen,hmax,p_hat),pinit,A,b,Aeq,beq,lb,ub); % minimize error on Y axis

% Plot estimated 2d curves
Pcat2d_fmc = catenary2D(rlen,hmax,p_fmc); % estimation from fmincon
Pcat2d_gna = catenary2D(rlen,hmax,p_gna); % estimation from gna
figure();
plot(xi,yi,'+') % sampled
hold on
plot(Pcat2d_fmc(2,:),Pcat2d_fmc(3,:),'r--') % fmincon
hold on
plot(Pcat2d_gna(2,:),Pcat2d_gna(3,:),'g--') % gna
hold on
l=legend('observed','Fmincon','Gauss-Newton');l.Location='best';
title('Estimating the catenary parameters')
xlabel('x')
ylabel('y')
% Plot estimated 3d curve
Pcat3d_fmc = catenary3D(rlen,hmax,p_fmc); % estimation from fmincon
Pcat3d_gna = catenary3D(rlen,hmax,p_gna); % estimation from gna

% Plot 3d curves
figure();
plot3(Pcat3d_fmc(1,:),Pcat3d_fmc(2,:),Pcat3d_fmc(3,:),'r--') % fmincon
hold on
plot3(Pcat3d_gna(1,:),Pcat3d_gna(2,:),Pcat3d_gna(3,:),'g--') % gna
l=legend('fmincon','Gauss-Newton');l.Location='best';
title('3D Catenary');
xlabel('x')
ylabel('y')
zlabel('z');
axis([-1.50 1.50 -1.50 1.50 -hmax 0.5])

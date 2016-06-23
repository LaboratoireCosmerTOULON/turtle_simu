% SCRIPT
%   Plot a catenary 3D curve and its projection on the image plane 
%   Estimate the catenary parameters from its projection
%   Unities : centimeters and radians

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
rlen    = 0.75;             % cable half-length in meters
hmax    = 0.9*rlen;         % cable maximum sag
cam_pc = [0 -0.11 0.28];   % position of rope attachment point (pc) in robot 1 in camera frame 
% Catenary parameters : vector p=(h/hmax,sin(theta))
s(1) = 0.8; % h = p(1)*hmax; cable sag
s(2) = 0.6;

% Draw 3D curve
Pcat3d = catenary3D(rlen,hmax,s,cam_pc);
x = Pcat3d(1,:);
y = Pcat3d(2,:);
z = Pcat3d(3,:);
% Calculate the projection
Pcat2d = catenaryProjection(rlen,hmax,s,x,y,z,cam_pc);
% Simulate a noisy observation
pob = 0.2; % percentage of observed curve
snr = 50; % signal noise ratio
Pcat2d_samp = catenarySampling(Pcat2d, pob, snr);
drawObjectiveFunction(Pcat2d_samp(1,:),Pcat2d_samp(2,:),rlen,hmax,cam_pc,1);

% Fitting with Gauss-Newton method
s_init = [0.5; 0.5]; % set initial guess
lb = [0.01; 0.01]; % lower bound
ub = [1; 1]; % upper bound
[s_gna,steps,chisq] = GaussNewton_v2(Pcat2d_samp(1,:),Pcat2d_samp(2,:),rlen,hmax,s_init,lb,ub,cam_pc); % minimization with dampled GNA
% Fitting with FMINCON
A = []; % -1 0; 1 0; 0 -1; 0 1
b = []; % 0; 50; 0; 1
Aeq = [];
beq = [];
s_fmc = fmincon(@(s_hat)fcout1(Pcat2d_samp(1,:),Pcat2d_samp(2,:),rlen,hmax,s_hat,cam_pc),s_init,A,b,Aeq,beq,lb,ub); % minimize error on Y axis
    
% Plot estimated 3d curve
% Rotate axis for better visualization in plot
rotm = eul2rotm([0 0 -pi/2]);
Pcat3d_plt = Pcat3d;
for i=1:length(Pcat3d)
    Pcat3d_plt(:,i) = rotm*Pcat3d(:,i);
end
Pcat3d_fmc  = catenary3D(rlen,hmax,s_fmc,cam_pc); % estimation from fmincon
Pcat3d_fmc_plt = Pcat3d_fmc;
for i=1:length(Pcat3d_plt)
    Pcat3d_fmc_plt(:,i) = rotm*Pcat3d_fmc(:,i);
end
Pcat3d_gna  = catenary3D(rlen,hmax,s_gna,cam_pc); % estimation from gna
Pcat3d_gna_plt = Pcat3d_gna;
for i=1:length(Pcat3d_plt)
    Pcat3d_gna_plt(:,i) = rotm*Pcat3d_gna(:,i);
end
%Pcat3d_gna  = rotm*Pcat3d_gna;
% Plot
figure();
plot3(Pcat3d_plt(1,:),Pcat3d_plt(2,:),Pcat3d_plt(3,:)) % original
hold on
plot3(Pcat3d_fmc_plt(1,:),Pcat3d_fmc_plt(2,:),Pcat3d_fmc_plt(3,:),'r--'); % fmincon
hold on
plot3(Pcat3d_gna_plt(1,:),Pcat3d_gna_plt(2,:),Pcat3d_gna_plt(3,:),'g--') % gna
l=legend('Real','fmincon','Gauss-Newton');l.Location='best';
title('3D Catenary');
xlabel('x');
ylabel('y');
zlabel('z');
% axis([-1.50 1.50 -1.50 1.50 -hmax 0.5])

% Plot estimated 2d curves
Pcat2d_fmc = catenaryProjection(rlen,hmax,s_fmc, Pcat3d_fmc(1,:), Pcat3d_fmc(2,:), Pcat3d_fmc(3,:),cam_pc); % estimation from fmincon
% Pcat2d_gna = catenaryProjection(rlen,hmax,s_gna, Pcat3d_gna(1,:), Pcat3d_gna(2,:), Pcat3d_gna(3,:),cam_pc); % estimation from gna
% Pcat2d_tst = catenaryProjection(rlen,hmax,s,Pcat2d_samp(1,:),cam_pc);
figure();
plot(Pcat2d(1,:),Pcat2d(2,:)) % original curve
hold on
plot(Pcat2d_samp(1,:),Pcat2d_samp(2,:),'+') % sampled
plot(Pcat2d_fmc(1,:),Pcat2d_fmc(2,:),'r--') % fmincon
% plot(Pcat2d_gna(1,:),Pcat2d_gna(2,:),'g--') % gna
% plot(Pcat2d_tst(1,:),Pcat2d_tst(2,:),'k--') % gna
l=legend('real','observed','estimated');l.Location='best'; %'Gauss-Newton','Validation'
title('Estimating the catenary parameters')
xlabel('x')
ylabel('y')
set(gca,'Ydir','reverse')
% axis([-1.50 1.50 -hmax 0.5])

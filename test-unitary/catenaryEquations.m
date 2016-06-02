% SCRIPT
%   Plot a catenary 3D curve and its projection on the image plane 
%   Estimate the catenary parameters from its projection
%   Unities : centimeters and radians

% Close all opened figures and clear workspace
close all;
clear; 
clc;

% Rope parameter
rlen = 0.75; % cable half-length in meters
hmax = 0.9*rlen; % cable maximum sag
% Catenary parameters : vector p=(h/hmax,sin(theta))
p(1) = 0.99; % h = p(1)*hmax; cable sag
p(2) = 0.4;

% Draw 3D curve
Pcat3d = catenary3D(rlen,hmax,p);
x = Pcat3d(1,:);
y = Pcat3d(2,:);
z = Pcat3d(3,:);
% Calculate the projection
Pcat2d = catenary2D(rlen,hmax,p);
% Simulate a noisy observation
pob = 0.75; % percentage of observed curve
snr = 50; % signal noise ratio
Pcat2d_samp = catenarySampling(Pcat2d, pob, snr);

% Fitting with Gauss-Newton method
pinit = [0.5; 0.5]; % set initial guess
lb = [0.01; 0.01]; % lower bound
ub = [1; 1]; % upper bound
[p_gna,steps,chisq] = GaussNewton_v2(Pcat2d_samp(1,:),Pcat2d_samp(2,:),rlen,hmax,pinit,lb,ub); % minimization with dampled GNA
% Fitting with FMINCON
A = []; % -1 0; 1 0; 0 -1; 0 1
b = []; % 0; 50; 0; 1
Aeq = [];
beq = [];
p_fmc = fmincon(@(p_hat)fcout1(Pcat2d_samp(1,:),Pcat2d_samp(2,:),rlen,hmax,p_hat),pinit,A,b,Aeq,beq,lb,ub); % minimize error on Y axis
    
% Plot estimated 2d curves
Pcat2d_fmc = catenary2D(rlen,hmax,p_fmc); % estimation from fmincon
Pcat2d_gna = catenary2D(rlen,hmax,p_gna); % estimation from gna
figure();
plot(Pcat2d(2,:),Pcat2d(3,:)) % original curve
hold on
plot(Pcat2d_samp(1,:),Pcat2d_samp(2,:),'+') % sampled
hold on
plot(Pcat2d_fmc(2,:),Pcat2d_fmc(3,:),'r--') % fmincon
hold on
plot(Pcat2d_gna(2,:),Pcat2d_gna(3,:),'g--') % gna
hold on
l=legend('real','observed','Fmincon','Gauss-Newton');l.Location='best';
title('Estimating the catenary parameters')
xlabel('x')
ylabel('y')
axis([-1.50 1.50 -hmax 0.5])
% Plot estimated 3d curve
Pcat3d_fmc = catenary3D(rlen,hmax,p_fmc); % estimation from fmincon
Pcat3d_gna = catenary3D(rlen,hmax,p_gna); % estimation from gna
figure();
plot3(Pcat3d(1,:),Pcat3d(2,:),Pcat3d(3,:)) % original
hold on
plot3(Pcat3d_fmc(1,:),Pcat3d_fmc(2,:),Pcat3d_fmc(3,:),'r--') % fmincon
hold on
plot3(Pcat3d_gna(1,:),Pcat3d_gna(2,:),Pcat3d_gna(3,:),'g--') % gna
l=legend('Real','fmincon','Gauss-Newton');l.Location='best';
title('3D Catenary');
xlabel('x')
ylabel('y')
zlabel('z');
axis([-1.50 1.50 -1.50 1.50 -hmax 0.5])

% % Estimate rope sag h from half-span D with McLaurin approximation
% C = 2*h/(rlen^2 - h^2);
% D = (1/C)*acosh(C*h + 1);
% h_hat = sqrt(rlen^2 -D^2);
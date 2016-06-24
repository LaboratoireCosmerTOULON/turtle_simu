% SCRIPT
%   Valider l'equation de projection de la chainette sur le plan image

% Close all opened figures and clear workspace
close all;
clear; 
clc;

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
s = [a; b];

% Constants for the catenary equation
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);

% Turtlebot features (measurements in SigmaC)
Tx      = 0.4;   % distance in x-axis between camera and rope attachment on robot
Ty      = -0.11; % distance in y-axis between camera and rope attachment on robot
Tz      = 0.28;  % distance in z-axis between camera and rope attachment on robot
Tcam    = [Tx Ty Tz];
f   = 1;      % focal length

% Rope 3D equation in Camera frame (SigmaC)
Pcat3d = catenary3D(rlen,hmax,s,Tcam);

% Standard projection
Pcat_proj   = catenaryProjection(rlen,hmax,s,Pcat3d(1,:),Pcat3d(2,:),Pcat3d(3,:),Tcam);
x_proj      = Pcat_proj(1,:); 
y_proj      = Pcat_proj(2,:); 

% Sampling projection and add some noise
pob = 1.0; % percentage of observed curve
snr = 10000; % signal noise ratio
Pcat_proj_samp = catenarySampling(Pcat_proj, pob, snr);
x_proj_samp = Pcat_proj_samp(1,:);

% Rope projection on image plan (equation de Vincent)
Pcat_proj_v = catenaryProjection_vincent(f,rlen,hmax,s,x_proj_samp,Tcam);
x_proj_v    = Pcat_proj_v(1,:);
y_proj_v    = Pcat_proj_v(2,:);

% Rope projection on image plan (equation de Matheus)
Pcat_proj_m = catenaryProjection_matheus(rlen,hmax,s,x_proj_samp,Tcam);
x_proj_m    = Pcat_proj_m(1,:);
y_proj_m    = Pcat_proj_m(2,:);

figure();
plot(x_proj,y_proj) % projection classique
hold on
plot(x_proj_v,y_proj_v,'r') % projection Vicnent
% plot(x_proj_m,y_proj_m,'g') % projection Matheus
l=legend('Projection p = f*P/Z','Projection Vincent)','Projection Matheus)');l.Location='best'; %'Gauss-Newton','Validation'
title('Catenary projection')
xlabel('x_{img} (m)')
ylabel('y_{img} (m)')
set(gca,'Ydir','reverse')




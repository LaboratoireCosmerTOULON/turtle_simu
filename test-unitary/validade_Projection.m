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

% Constants for the catenary equation
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);

% Turtlebot features (measurements in SigmaC)
Tx  = 0.0;   % distance in x-axis between camera and rope attachment on robot
Ty  = -0.11; % distance in y-axis between camera and rope attachment on robot
Tz  = 0.28;  % distance in z-axis between camera and rope attachment on robot
f   = 1;      % focal length

% Rope 3D equation in Camera frame (SigmaC)
t           = linspace(-D,D,1000);
sigmaC_X    = -b.*(t + D) + Tx;
sigmaC_Y    = -1/C.*(cosh(C*t) - 1) + h + Ty;
sigmaC_Z    = sqrt(1-b^2)*(t + D) + Tz;

% Standard projection
x_proj = sigmaC_X./sigmaC_Z;
y_proj = sigmaC_Y./sigmaC_Z;

% Rope projection on image plan (equation de Vincent)
xi = x_proj;
qx  = (f*(Tx - D*b) - xi*(Tz + D*sqrt(1-b^2)))./(xi*sqrt(1-b^2) + f*b);
yi   = f*(-1/C*(cosh(C*qx) - 1) + h + Ty)./(sqrt(1-b^2)*qx + D*sqrt(1-b^2) + Tz);

% Rope projection on image plan (equation de Matheus)
z = Tz./(1 + xi*(sqrt(1-b^2)/b));
t = -(Tz*xi)./(b + xi*sqrt(1-b^2));
yi2 = (1./z).*(-(1/C)*(cosh(C*(t-D))-1) + h + Ty);

figure();
plot(x_proj,y_proj) % projection classique
hold on
plot(xi,yi,'r') % projection Vicnent
plot(xi,yi2,'g') % projection Matheus
l=legend('Projection p = f*P/Z','Projection Vincent)','Projection Matheus)');l.Location='best'; %'Gauss-Newton','Validation'
title('Catenary projection')
xlabel('x_{img} (m)')
ylabel('y_{img} (m)')
set(gca,'Ydir','reverse')




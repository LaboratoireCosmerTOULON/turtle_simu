function P = catenary3D_Hugel(rlen,hmax,s, Tc)
% function for calculation of a 3D catenary equation in the camera
% frame
% INPUTS :
%   rlen  = rope half-length
%   hmax  = rope maximum sag
%   s     = catenary parameters :
%           s(1)    = sag ratio h/hmax
%           s(2)    = sinus of curve angle with robot azimuth
%   pc    = rope attachement point in camera frame
% OUTPUTS :
%   P = [x;y;z] = output matrix with catenary coordinates

a = s(1); % h/hmax
b = s(2); % sin(psi)
h = a*hmax;
Tc_x = Tc(1);
Tc_y = Tc(2);
Tc_z = Tc(3);

C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);
t = linspace(-D,D,1001);
xc = -b*(t+D) + Tc_x;
yc = -(1/C)*(cosh(C*t)-1) + h + Tc_y;
zc = sqrt(1-b^2)*(t+D) + Tc_z;
P = [xc; yc; zc];
end
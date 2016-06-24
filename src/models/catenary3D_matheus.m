function P = catenary3D_matheus(rlen,hmax,s, pc)
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

% ************ ATTENTION **************** %
% THIS EQUATION IS NOT VALID. THIS IS AN OLD VERSION OF THE FUNCTION

a = s(1); % h/hmax
b = s(2); % sin(psi)
h = a*hmax;
pc_x = pc(1);
pc_y = pc(2);
pc_z = pc(3);

C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);
t = linspace(-D,D,1001);
x = -b*(t+D) + pc_x;
z = sqrt(1-b^2)*(t+D) + pc_z;
y = -(1/C)*(cosh(C*(-x./b-D))-1) + h + pc_y; % catenary
P = [x;y;z];
end
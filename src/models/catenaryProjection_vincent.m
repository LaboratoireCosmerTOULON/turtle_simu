function P = catenaryProjection_vincent(varargin)
% catenary2D calculates the 2D catenary equation in frame pC, the rope
% attachment point on robot r2
% Inputs
%   varargin{1} = rlen    :    rope half-length
%   varargin{2} = hmax    :    rope maximum sag
%   varargin{3} = s       :    catenary parameters
%                 s(1) : h/hmax
%                 s(2) : sin(theta)
%   varargin{4} = x       :    row-vector with catenary 2d coordinates
% Outputs
%   [ x; y ]: matrix with catenary coordinates

% Extract fucntion arguments
f       = varargin{1};
rlen    = varargin{2};
hmax    = varargin{3};
s       = varargin{4};
a       = s(1);
b       = s(2);
% Some constantes for catenary equation
h = a*hmax;
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);

% If the user provides the rope 3D coordinates in the camera frame we can
% use the relation x_img = x/z and y_img = y_z
% Otherwise, if the user provides the vector x_img, use it to calculate
% y_img
x_img = zeros(1,1);
y_img = zeros(1,1);
if(nargin == 8)
    x = varargin{5}; % x-coordinate of catenary in camera frame
    y = varargin{6}; % y-coordinate of catenary in camera frame
    z = varargin{7}; % z-coordinate of catenary in camera frame
    % pc = varargin{7};% not used
    x_img = x./z;
    y_img = y./z;
elseif(nargin == 6)
    x_img = varargin{5}; % x-coordinate of catenary in image plane
    Tc = varargin{6};    % rope attachment point 
    Tc_x = Tc(1);
    Tc_y = Tc(2);
    Tc_z = Tc(3);
    t = (f*(Tc_x - D*b) - x_img*(Tc_z + D*sqrt(1-b^2)))./(x_img*sqrt(1-b^2) + f*b);
    y_img = f*(-1/C*(cosh(C*t) - 1) + h + Tc_y)./(sqrt(1-b^2)*(t + D) + Tc_z);
else 
    disp('Error: worng number or arguments in function catenaryProjection');
end
% Calculate the catenary projection in the image plane considering f=1m
P = [x_img; y_img; ones(1,length(x_img))];
end

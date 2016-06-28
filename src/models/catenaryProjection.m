function P = catenaryProjection(varargin)
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
rlen = varargin{1};
hmax = varargin{2};
s = varargin{3};
a = s(1);
b = s(2);
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
if(nargin == 7)
    x = varargin{4}; % x-coordinate of catenary in camera frame
    y = varargin{5}; % y-coordinate of catenary in camera frame
    z = varargin{6}; % z-coordinate of catenary in camera frame
    % pc = varargin{7};% not used
    x_img = x./z;
    y_img = y./z;
elseif(nargin == 5)
    x_img = varargin{4}; % x-coordinate of catenary in image plane
    Tcam = varargin{5};    % rope attachment point 
    Tx = Tcam(1);
    Ty = Tcam(2);
    Tz = Tcam(3);
    z = (Tx*sqrt(1-b^2) + Tz*b)./(b + x_img*(sqrt(1-b^2)));
    t = (Tx - Tz*x_img)./(b + x_img*sqrt(1-b^2));
    y_img = (1./z).*(-(1/C)*(cosh(C*(t-D))-1) + h + Ty);
else 
    disp('Error: worng number or arguments in function catenaryProjection');
end
% Calculate the catenary projection in the image plane considering f=1m
P = [x_img; y_img; ones(1,length(x_img))];
end

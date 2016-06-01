function P = catenary2D(varargin)
% catenary2D calculates the 2D catenary equation in frame pC, the rope
% attachment point on robot r2
% Inputs
%   varargin{1} = rlen    :    rope half-length
%   varargin{2} = hmax    :    rope maximum sag
%   varargin{3} = p       :    catenary parameters
%                 p(1) : h/hmax
%                 p(2) : sin(theta)
%   varargin{4} = x       :    row-vector with catenary 2d coordinates
% Outputs
%   [ x; y ]: matrix with catenary coordinates

% Extract fucntion arguments
rlen = varargin{1};
hmax = varargin{2};
p = varargin{3};
a = p(1);
b = p(2);
% Some constantes for catenary equation
h = a*hmax;
C = 2*h/(rlen^2 - h^2);
D = (1/C)*acosh(C*h + 1);

if(nargin > 3) % if vector x is provided, use it, otherwise crete it
    y = varargin{4};
else
    y = linspace(0,(2*D)*b,1001);
end
% Calculate z vector
x = zeros(1,length(y));
z = (1/C)*(cosh(C*((y./b)-D))-1)-h;
P = [x; y; z];
end

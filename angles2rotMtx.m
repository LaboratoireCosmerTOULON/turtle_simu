function [ Rot ] = angles2rotMtx( angles )
% rotMtx Generates a rotation matrix from three Cardan angles phi, theta and
% psi in radians
%   phi     :   rotation around X-axis (roll)
%   theta   :   rotation around Y-axis (pitch)
%   psi     :   rotation around Z-axis (yaw)
% Reference :   Section 43.2.1, Handbook of Robotics (Springer) and 

phi = angles(1);
theta = angles(2);
psi = angles(3);

Rotx = [ 1     0       0
         0  cos(phi) sin(phi)
         0 -sin(phi) cos(phi) ];

Roty = [ cos(theta) 0 -sin(theta)
             0      1     0 
         sin(theta) 0  cos(theta)];

Rotz = [ cos(psi) sin(psi) 0
        -sin(psi) cos(psi) 0
            0       0       1 ];
        
Rot = Rotx*Roty*Rotz;


% M = [               cos(psi)*cos(theta),                                 sin(psi)*cos(theta),                 -sin(theta)     ;
%       -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi),   cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi),  sin(phi)*cos(theta) ;
%        sin(psi)*sin(phi)+cos(psi)*sin(theta)*cos(phi),  -cos(psi)*sin(phi)+sin(psi)*sin(theta)*cos(phi),  cos(phi)*cos(theta)  ];
end


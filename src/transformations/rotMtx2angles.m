function [ angles ] = rotationMatrix2angles( Rot )
% rotMtx2angles retieves the Cardan angles from a given rotation matrix
% Reference: J Diebel, Stanford 2006

phi   = atan2(Rot(2,3), Rot(3,3));
theta = -asin(Rot(1,3));
psi   = atan2(Rot(1,2), Rot(1,1));

angles = [phi; theta; psi];
end


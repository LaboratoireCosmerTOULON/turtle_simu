function [ M ] = transf_W2R( R, Jo )
% tranfMtx Generates the transformation matrix for a given rotation and translation
%   R       :   rotation matrix World-to-Robot for linear velocity transformation
%   Jo      :   jacobian matrix World-to-Robot for angular velocity
%               transformation
% Reference :   Section 43.2.1, Handbook of Robotics (Springer)

Null = zeros(3,3);
M = [R Null; Null Jo];
end


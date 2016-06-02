function [ M ] = pose2homogeneousMatrix( pose )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% Translation
Tr = [pose(1);pose(2);pose(3)];

% Rotation
Rot = angles2rotationMatrix(pose(4:6));


% Corresponding homogeneous matrix
M = [   Rot,    Tr;
     zeros(3,1), 1];

end


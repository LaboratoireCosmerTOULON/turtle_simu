function [ V ] = tiwstMatrix( M )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

V = M;
t = M(1:3,4); % translation
% skew t
tx = skew(t);
% extract the rotation
Rot = M(1:3,1:3);
% compute the twist matrix
V = [Rot  tx*Rot; zeros(3,3) Rot];
end


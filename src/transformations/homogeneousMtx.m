function [ M ] = homogeneousMtx( Rot, T )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

M = [Rot T; zeros(1,3) 1];
end


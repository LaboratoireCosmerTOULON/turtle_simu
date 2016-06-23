% SCRIPT
%   Plot a catenary 3D curve and its projection on the image plane 
%   Estimate the catenary parameters from its projection
%   Unities : centimeters and radians

% Close all opened figures and clear workspace
close all;
clear; 
clc;

addpath /home/matheus/Documents/MATLAB/turtle_simu/src/data
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/drawing
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/estimation
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/optimization
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/sampling

% Rope parameter
rlen    = 0.75;             % cable half-length in meters
hmax    = 0.9*rlen;         % cable maximum sag


% Rope equation in frame Sigma1
sig1_x = linspace(-D:D);

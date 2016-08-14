% Draw curve
close all;
clear all;
clc;

fileID = fopen('/home/matheus/catkin_ws/brouillons/Sgna.dat','r');
formatSpec = '%f %f\n';
sizeA = [1250 2];
A = dlmread('/home/matheus/catkin_ws/brouillons/Sgna.dat');

figure()
plot(1:1051,asin(A(1:1051,2))*180/pi);
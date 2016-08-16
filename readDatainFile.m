clear all;
close all;
clc;

V = dlmread('/home/matheus/catkin_ws/brouillons/Vcmd.dat');
S = dlmread('/home/matheus/catkin_ws/brouillons/Sgna.dat');
Se = dlmread('/home/matheus/catkin_ws/brouillons/Se.dat');

nMesurements = 195;
k = 1:nMesurements;
ad = 0.8;
bd =-0.8;

figure();
plot(k,V(k,1));
hold on
plot(k,V(k,2),'r');
l=legend('vx','wz');l.Location='best';
title('Robot Velocity')
xlabel('Sample')
ylabel('Velocity (m/s or rad/s)')

figure();
plot(k,S(k,1),'.');
hold on
plot(k,S(k,2),'.r');
plot(k,Se(k,1));
plot(k,Se(k,2),'r');
plot(k,ad,'k');
plot(k,bd,'k');
l=legend('a=h/hmax','b=sin(theta)','a-a*','b-b*');l.Location='best';
title('Parameter Evolution')
xlabel('Sample')
ylabel('Catenary parameter')

% Simulation of Image-Based Visual Servoing (IBVS)
% Application : Tow turtlebots (r1 and r2) are linked by a rope. The turtle
% T2 is visually guided by the rope in order to follow the leader (r1).
% This script simulates the IBVS of turtle r2 by the rope

close all;
clear all;
clc;
addpath ('src/transformations', ...
         'src/estimation',...
         'src/drawing',...
         'src/models',...
         'src/sampling',...
         'src/visualServoing',...
         'src/optimization');




% Rope parameters
rlen = 0.75; % rope half-length in meters
hmax = 0.9*rlen; % rope maximum possible sag

% Constants for estimating the catenary rope model. The rope is modeled by
% a catenary with parameters a = h/hmax and b = sin(theta), s = (a,b). h is
% the current rope sag and theta is the angle between the rope and the
% robot r2 XZ plane
s_init = [0.5; 0.5]; % set initial guess for catenary parameters estimation
lb = [0.01; 0.01]; % params lower bound
ub = [1; 1]; % params upper bound

% Constants for visual servoing
a_d = 0.3;
theta_d = 5; % in degres
b_d = sin(deg2rad(theta_d));
s_d = [a_d; b_d];

% World
sigma_W = [0; 0; 0; 0; 0; 0]; % World frame (meters and radians)
% Robot
turtle_radius = 0.2;
w_p_r1 = [2.5; 2; 0; 0; 0; 0]; % initial position of turtle r1 in World frame
w_p_r2 = [1.5; 1; 0; 0; 0; deg2rad(-15)]; % initial position of turtle r2 in World frame
% Rope
r1_p_ropeA = [-turtle_radius; 0; 0; 0; 0; 0]; % rope attachment point (pA) on robot r1 in r1-frame
r2_p_ropeC = [turtle_radius; 0; 0; 0; 0; 0]; % rope attachment point (pC) on robot r2 in r2-frame
w_p_ropeA  = zeros(6,1); % Init the rope pose

% Camera
r2_p_cam = [-turtle_radius; 0; 0; 0; 0; 0]; % position of camera embeeded on turtle r2 oin r2-frame

% Start simualtion
dt = 0.01; % in seconds
t_end = 5;
r1_v_r1 = [0; 0; 0; 0; 0; deg2rad(0)]; % velocity of r1 in robot frame
r2_v_r2 = [0; 0; 0; 0; 0; deg2rad(0)]; % velocity of r2 in robot frame

fig = figure('units','normalized','outerposition',[0 0 1 1]);
r2_v_r2_h = r2_v_r2;
for t = 0:dt:t_end
    % clear figure
    clf(fig);

    % Motion equation for robot r1
    r1_R_w = angles2rotMtx(w_p_r1(4:6)); % rotation matrix World-to-robot
    r1_J_w = angularRateMtx(w_p_r1(4:6)); % World-to-robot
    w_T_r1 = transf_W2R(r1_R_w',inv(r1_J_w)); % Inverse transformation: R2W
    w_p_ropeA = w_p_ropeA + w_T_r1*r1_p_ropeA; % position of pA in World-frame
    
    % Motion equation for robot r2
    r2_R_w = angles2rotMtx(w_p_r2(4:6)); % rotation matrix World-to-robot
    r2_J_w = angularRateMtx(w_p_r2(4:6)); % World-to-robot
    w_T_r2 = transf_W2R(r2_R_w',inv(r2_J_w)); % Inverse transformation: R2W
    w_p_ropeC = w_p_r2 + w_T_r2*r2_p_ropeC; % position of pA in World-frame
    w_p_cam = w_p_r2 + r2_p_cam; % position of pA in World-frame
    
    % Retrive rope parameters
    [a, b] = retrieveCatenaryParams(rlen, hmax, w_p_ropeA, w_p_ropeC, w_p_r2);

    % draw scene
    drawScene(fig, w_p_r1, w_p_r2, turtle_radius, w_p_ropeA, w_p_ropeC);
    % draw 2d catenary 
    Pcat2d_samp = draw2dCatenary(fig, w_p_cam, w_p_ropeC, rlen, hmax, [a;b]);
    
    % perform visual servoing
    [ v_pC, s_gna ] = getVelocity(fig, Pcat2d_samp, rlen, hmax, s_d, s_init, lb, ub);
    r2_v_r2(1) = v_pC(1);
    r2_v_r2(6) = v_pC(6);
    r2_v_r2_h = [r2_v_r2_h, r2_v_r2];
    % Robots motion...
    w_p_r1 = w_p_r1 + r1_v_r1*dt; % motion of r1 in World-frame
    w_p_r2 = w_p_r2 + w_T_r2*r2_v_r2*dt; % motion of r2 in World-frame

    pause(dt)
end

% Result analysis
time = 0:dt:t_end; % simulation time
% Plot velocity
figure();
plot(time,r2_v_r2_h(1,2:end),'k');
hold on;
plot(time,r2_v_r2_h(2,2:end),'b');
plot(time,r2_v_r2_h(3,2:end),'g');
plot(time,r2_v_r2_h(4,2:end),'y');
plot(time,r2_v_r2_h(5,2:end),'m');
plot(time,r2_v_r2_h(6,2:end),'r');
% plot options
title('Plot velocity')
xlabel('time (s)')
ylabel('velocity (m/s) or (rad/s)')
l=legend('vx','vy','vz','wx','wy','wz');
l.Location='best';

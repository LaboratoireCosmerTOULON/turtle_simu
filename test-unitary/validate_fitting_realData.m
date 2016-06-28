% SCRIPT
%   Plot a catenary 3D curve and its projection on the image plane 
%   Estimate the catenary parameters from its projection
%   Unities : centimeters and radians

% Close all opened figures and clear workspace
close all;
clear; 
clc;    

addpath /home/matheus/Documents/MATLAB/turtle_simu/src/data
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/estimation
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/optimization
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/sampling

% Read measurements

% Read image
for k=200:10:450;
    imagename = sprintf('/home/matheus/catkin_ws/imagesturtle/imageturtle_%d.jpeg',k);
    I = imread(imagename);
    figure, imshow(I);
    
    % Find white pixels
    [vi,ui] = find(I == 1); % [row,col] = find()
    ui = ui';
    vi = vi';
    % camera parameters
    uo = 319.5;
    vo = 239.5;
    px = 525.0;
    py = 525.0;
    % transform from pixels to meters
    xi = (ui-uo)./px;
    yi = (vi-vo)./py;
    
    % print
    %     for l=1:length(ui)
    %         fprintf('I_%d : (u=%d v=%d) (x=%f y=%f) \n', k, ui(l), vi(l), xi(l), yi(l));
    %     end
    
    % Rope parameter
    rlen = 0.48; % cable half-length in meters
    hmax = 0.40; % cable maximum sag
    
    % Turtlebot features (measurements in SigmaC)
    Tx      = 0.0;          % distance in x-axis between camera and rope attachment on robot
    Ty      = -0.11;        % distance in y-axis between camera and rope attachment on robot
    Tz      = 0.28;         % distance in z-axis between camera and rope attachment on robot
    Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame
    
    % Fitting with Gauss-Newton method
    s_init = [0.5; 0.5]; % set initial guess
    lb = [0.01; 0.01]; % lower bound
    ub = [1; 1]; % upper bound
    % [p_gna,steps,chisq] = GaussNewton_v2(xi,yi,rlen,hmax,pinit,lb,ub); % minimization with dampled GNA
    % Fitting with FMINCON
    A = [];
    b = [];
    Aeq = [];
    beq = [];
    s_fmc = fmincon(@(s_hat)fcout1(xi,yi,rlen,hmax,s_hat,Tcam),s_init,A,b,Aeq,beq,lb,ub); % minimize error on Y axis
    % drawObjectiveFunction(xi,yi,rlen,hmax,Tcam,1);
    
    fprintf('I_%d : h= %f, theta= %f \n', k, hmax*s_fmc(1), asin(s_fmc)*180/pi);
    
%     % Plot estimated 2d curves
%     Pcat3d_fmc  = catenary3D(rlen,hmax,s_fmc,Tcam); % estimation from fmincon
%     Pcat2d_fmc = catenaryProjection(rlen,hmax,s_fmc, Pcat3d_fmc(1,:), Pcat3d_fmc(2,:), Pcat3d_fmc(3,:),Tcam); % estimation from fmincon
%     % Pcat2d_gna = catenary2D(rlen,hmax,p_gna); % estimation from gna
%     figure();
%     plot(xi,yi,'+') % sampled
%     hold on
%     plot(Pcat2d_fmc(1,:),Pcat2d_fmc(2,:),'r--') % fmincon
%     hold on
%     % plot(Pcat2d_gna(2,:),Pcat2d_gna(3,:),'g--') % gna
%     % hold on
%     l=legend('observed','Fmincon');l.Location='best';
%     title('Estimating the catenary parameters')
%     xlabel('x')
%     ylabel('y')
%     set(gca,'Ydir','reverse')
%     axis 'equal'
    
    % % Plot estimated 3d curve
    % Pcat3d_fmc = catenary3D(rlen,hmax,s_fmc); % estimation from fmincon
    % % Pcat3d_gna = catenary3D(rlen,hmax,p_gna); % estimation from gna
    %
    % % Plot 3d curves
    % figure();
    % plot3(Pcat3d_fmc(1,:),Pcat3d_fmc(2,:),Pcat3d_fmc(3,:),'r--') % fmincon
    % % hold on
    % % plot3(Pcat3d_gna(1,:),Pcat3d_gna(2,:),Pcat3d_gna(3,:),'g--') % gna
    % l=legend('fmincon');l.Location='best';
    % title('3D Catenary');
    % xlabel('x')
    % ylabel('y')
    % zlabel('z');
    % axis([-1.50 1.50 -1.50 1.50 -hmax 0.5])
    
end


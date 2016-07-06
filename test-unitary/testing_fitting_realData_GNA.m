% SCRIPT
%   Plot a catenary 3D curve and its projection on the image plane 
%   Estimate the catenary parameters from its projection
%   Unities : centimeters and radians

% Close all opened figures and clear workspace
close all;
clear; 
clc;    
options = optimoptions('fmincon','Display','off');

addpath /home/matheus/Documents/MATLAB/turtle_simu/src/data
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/drawing
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/estimation
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/models
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/optimization
addpath /home/matheus/Documents/MATLAB/turtle_simu/src/sampling


%     ANGLE      #IMG
% -75 degrees : 1 - 35
% -60 degrees : 36 - 70
% -45 degrees : 71 - 130
% -30 degrees : 131 - 180
% -15 degrees : 181 - 210
%  15 degrees : 211 - 260
%  30 degrees : 261 - 310
%  45 degrees : 311 - 390
%  60 degrees : 391 - 440
%  75 degrees : 441 - 482

Svec_gna = zeros(2,482);
Svec_fmc = zeros(2,482);
Svec_real = zeros(2,482);

% Read image
prompt = 'What is the number of the first image to be read?';
ko = input(prompt);

for k=ko:482;
    
    angle = 0;
    if(k < 36)
        angle = -75;
    elseif(k >= 36 && k < 71)
        angle = -60;
    elseif(k >= 71 && k < 131)
        angle = -45;
    elseif(k >= 131 && k < 181)
        angle = -30;
    elseif(k >= 181 && k < 211)
        angle = -15;
    elseif(k >= 211 && k < 261)
        angle = 15;
    elseif(k >= 261 && k < 311)
        angle = 30;
    elseif(k >= 311 && k < 391)
        angle = 45;
    elseif(k >= 391 && k < 440)
        angle = 60;
    else
        angle = 75;
    end
    
    imagename = sprintf('/home/matheus/catkin_ws/imagesturtle/imgMap/imageturtle_%d.jpeg',k);
    I = imread(imagename);
    I = im2bw(I, 0.9);
    
    % Find white pixels
    [vi,ui] = find(I); % [row,col] = find()
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
    % for plotting
    xp = xi;
    yp = yi;
    
    % If theta is negative, invert xi
    inv = false;
    if(mean(xi) > 0)
        xi = -xi;
        inv = true;
    end
    
    
    % print
    %     for l=1:length(ui)
    %         fprintf('I_%d : (u=%d v=%d) (x=%f y=%f) \n', k, ui(l), vi(l), xi(l), yi(l));
    %     end
    
    % Rope parameter
    rlen = 0.48; % cable half-length in meters
    hmax = 0.40; % cable maximum sag
    
    % Turtlebot features (measurements in SigmaC)
    Tx      = -0.01;          % distance in x-axis between camera and rope attachment on robot
    Ty      = -0.11;        % distance in y-axis between camera and rope attachment on robot
    Tz      =  0.27;         % distance in z-axis between camera and rope attachment on robot
    Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame
    
    % Fitting with Gauss-Newton method
    s_init = [0.5; 0.5]; % set initial guess
    lb = [0.01; 0.01]; % lower bound
    ub = [1; 1]; % upper bound
    [s_gna,steps,chisq] = GaussNewton_v2(xi,yi,rlen,hmax,s_init,lb,ub,Tcam); % minimization with dampled GNA
    
    % Calculate catenary from estimated parameters
    np3d = 1000;
    Pcat3d_gna  = catenary3D(rlen,hmax,s_gna,Tcam,np3d); % estimation from fmincon
    Pcat2d_gna = catenaryProjection(rlen,hmax,s_gna, Pcat3d_gna(1,:), Pcat3d_gna(2,:), Pcat3d_gna(3,:),Tcam); % estimation from gna
    
    if(inv == true)
        s_gna(2)        = -s_gna(2);
        Pcat2d_gna(1,:) = -Pcat2d_gna(1,:);
    end
    
    Svec_real(:,k) = [0.20; angle];
    Svec_gna(:,k) = [hmax*s_gna(1); asin(s_gna(2))*180/pi];
    %     fprintf('I_%d (theta=%d)\nGNA=(%f,%f)\n', k, angle, hmax*s_gna(1), asin(s_gna(2))*180/pi);    
    
 
    % Plot results
    clf; % clear figures before plotting
    % Show image
    figure(1);
    imshow(I);
    % Plot estimation
    figure(2);
    plot(xp,yp,'.');
    hold on
    plot(Pcat2d_gna(1,:),Pcat2d_gna(2,:));
    l=legend('Acquired','GNA');l.Location='best';
    title('Estimating sag from real data')
    xlabel('x_{img}(m)')
    ylabel('y_{img}(m)')
    set(gca,'Ydir','reverse')
    axis([-0.64 0.64 -0.48 0.48])
    fprintf('I_%d (theta=%d)\nGNA=(%f,%f)\n', k, angle, hmax*s_gna(1), asin(s_gna(2))*180/pi);
    
    % Plot objective fucntion
%     axismax = 10000;
%     drawObjectiveFunction(xi,yi,rlen,hmax,Tcam,axismax,2);

    % Wait to continue
    w = waitforbuttonpress;
%     pause(1);
    
end

% Plot estimated angle
figure();
plot(1:482,Svec_real(1,:))
hold on
plot(1:482,Svec_gna(1,:))
l=legend('real','GNA');l.Location='best';
title('Estimating sag from real data')
xlabel('k')
ylabel('Rope sag (m)')

% Plot estimated angle
figure();
plot(1:482,Svec_real(2,:))
hold on
plot(1:482,Svec_gna(2,:))
l=legend('real','GNA');l.Location='best';
title('Estimating angle from real data')
xlabel('k')
ylabel('Angle(degrees)')
    
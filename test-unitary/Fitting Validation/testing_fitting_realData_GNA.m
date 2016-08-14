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


% Read image
prompt = 'What is the number of the first image to be read?';
ko = input(prompt);
prompt = 'Display partial results while running? (1/0)';
dp = input(prompt);
nImages = 1250;

% Rope parameter
rlen = 0.70; % cable half-length in meters
hmax = 0.40; % cable maximum sag
% Turtlebot features (measurements in SigmaC)
Tx      = -0.01;          % distance in x-axis between camera and rope attachment on robot
Ty      = -0.11;        % distance in y-axis between camera and rope attachment on robot
Tz      =  0.24;         % distance in z-axis between camera and rope attachment on robot
Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame
% Camera parameters
uo = 319.5;
vo = 239.5;
px = 525.0;
py = 525.0;

% Vector with estimated parameters
Svec_gna = zeros(2,nImages);
Svec_real = zeros(2,nImages);
minPoints = 3500; % minimum of points to an image be considered as valid
rho_min   = 0.85;

for k=ko:nImages;
    
    angle = 0;
    if(k < 144)
        angle = -75;
    elseif(k >= 144 && k < 235)
        angle = -60;
    elseif(k >= 235 && k < 355)
        angle = -45;
    elseif(k >= 355 && k < 557)
        angle = -30;
    elseif(k >= 557 && k < 649)
        angle = -15;
    elseif(k >= 649 && k < 738)
        angle = 0;
    elseif(k >= 738 && k < 861)
        angle = 15;
    elseif(k >= 861 && k < 971)
        angle = 30;
    elseif(k >= 971 && k < 1084)
        angle = 45;
    elseif(k >= 1084 && k < 1212)
        angle = 60;
    else
        angle = 75;
    end
    
    imagename = sprintf('/home/matheus/catkin_ws/brouillons/imgRope/imgMap/imgBW_%d.jpeg',k);
    I = imread(imagename);
    I = im2bw(I, 0.9);
    
    % Find white pixels
    [vi,ui] = find(I); % [row,col] = find()
    ui = ui';
    vi = vi';
    % transform from pixels to meters
    xi = (ui-uo)./px;
    yi = (vi-vo)./py;
    % for plotting
    xp = xi;
    yp = yi;
    
    % Calculate Pearson correlation coefficient
    rho = corrcoef(xi',yi');
    % Only process image if enough points and Pearson coeff.
    if(numel(xi) > minPoints && abs(rho(2)) > rho_min)
        % If theta is negative, invert xi
        inv = false;
        if(mean(xi) > 0)
            xi = -xi;
            inv = true;
            Tx      = -0.01;          % distance in x-axis between camera and rope attachment on robot
            Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame
        else
            Tx      = -0.03;          % distance in x-axis between camera and rope attachment on robot
            Tcam    = [Tx Ty Tz];   % Translation between rope attachment point and camera frame
        end
        
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
        
        % Save results in vector
        da = 0;
        db = 0;
        if(k-ko > 1)
            da = abs(s_gna(1) - Svec_gna(1,k-1));
            db = abs(s_gna(2) - Svec_gna(2,k-1));
            if( da < 0.5 && db < 0.5 )
                Svec_real(:,k) = [0.20; angle];
                Svec_gna(:,k) = s_gna(:);
            else
                Svec_real(:,k) = [0.20; angle];
                Svec_gna(:,k) =  Svec_gna(:,k-1);
            end
        else
            Svec_real(:,k) = [0.20; angle];
            Svec_gna(:,k) = s_gna(:);
        end

        
        % Plot results
        if(dp)
            clf; % clear figures before plotting
            % Show image
            figure(1);
            imshow(I);
            % Plot estimation
            figure(2);
            plot(xp,yp,'.');
            hold on
            plot(Pcat2d_gna(1,:),Pcat2d_gna(2,:),'r');
            l=legend('Acquired','GNA');l.Location='best';
            title('Estimating sag from real data')
            xlabel('x_{img}(m)')
            ylabel('y_{img}(m)')
            set(gca,'Ydir','reverse')
            axis([-0.64 0.64 -0.48 0.48])
            fprintf('I_%d (theta=%d)\nGNA=(%f,%f)\tsteps %d, chisq %f\n', k, angle, hmax*s_gna(1), asin(s_gna(2))*180/pi,steps,chisq);
            
            % Wait to continue
            w = waitforbuttonpress;
            %     pause(0.1);
        end
    elseif(k-ko >= 1)
        Svec_real(:,k) = [0.20; angle];
        Svec_gna(:,k) = Svec_gna(:,k-1);
    else
        Svec_real(:,k) = [0.20; angle];
        Svec_gna(:,k) = [0/0; 0/0];
    end
    
end

% Plot estimated angle
figure();
plot(1:nImages,Svec_real(1,:))
hold on
plot(1:nImages,hmax*Svec_gna(1,:),'r')
l=legend('real','GNA');l.Location='best';
title('Estimating sag from real data')
xlabel('k')
ylabel('Rope sag (m)')

% Plot estimated angle
figure();
plot(1:nImages,Svec_real(2,:))
hold on
plot(1:nImages,asin(Svec_gna(2,:))*180/pi,'r')
l=legend('real','GNA');l.Location='best';
title('Estimating angle from real data')
xlabel('k')
ylabel('Angle(degrees)')
    
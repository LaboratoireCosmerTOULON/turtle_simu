% SCRIPT
%   Unitary teste for validating fitting using fmincon and GNA

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


% Global variable in this script
% File name format to same graphs of parameters estimation erros
filenameformat = '/Figures/160623_fcost_analysis/eparam_fmincon_%d.fig';
% Observation parameters
snr = 100; % signal noise ratio
nob = 100; % number of samples
pob_c = 0.1:0.05:0.2; % percentage of observed catenary
% optimization options
options = optimoptions('fmincon','Display','off');

% Rope parameter
rlen    = 0.75;             % cable half-length in meters
hmax    = 0.9*rlen;         % cable maximum sag
Tcam = [0 -0.11 0.28];   % position of rope attachment point (pc) in robot 1 in camera frame 

k = 1; % counter
[X,Y] = meshgrid(0.05:0.05:1, 0.05:0.05:1);
sz = size(X);
Z_e1 = zeros(sz);
Z_e2 = zeros(sz);
Z_e3 = zeros(sz);
mZ_e1 = zeros(length(pob_c),1);
steps_k = zeros(length(pob_c),1);


for k=1:length(pob_c) % index for selecting percentage of observed curve
    for i=1:sz(1) % index of vector of parameters
        for j=1:sz(2) % index of vector of parameters
            
            pob = pob_c(k); % percentage of the observed curve
            s(1) = X(1,j);
            s(2) = Y(i,1);
            fprintf('pob=%.2f, s=[%.2f %.2f]) \n',pob,s);
            
            % Calculate observed catenary 3D form
            Pcat3d = catenary3D(rlen,hmax,s,Tcam);
            x = Pcat3d(1,:);
            y = Pcat3d(2,:);
            z = Pcat3d(3,:);
            % Calculate the projection
            Pcat2d = catenaryProjection(rlen,hmax,s,x,y,z,Tcam);
            % Simulate a noisy observation
            Pcat2d_samp = catenarySampling(Pcat2d, pob, snr);
            x_ob = Pcat2d_samp(1,:);
            y_ob = Pcat2d_samp(2,:);
            
            % Fitting with FMINCON
            % calculate the parameters that minimize the cost function
            s_init = [0.9; 0.9]; % set initial guess
            lb = [0.01; 0.01]; % lower bound
            ub = [1; 1]; % upper bound
            A = [];
            b = [];
            Aeq = [];
            beq = [];
            s_fmc = fmincon(@(s_hat)fcout1(x_ob,y_ob,rlen,hmax,s_hat,Tcam),s_init,A,b,Aeq,beq,lb,ub,[],options);
            
            % estimation errors with FMINCON
            Z_e1(i,j) = (1/sqrt(2))*sqrt((X(1,j) - s_fmc(1))^2 + (Y(i,1) - s_fmc(2))^2);
        end
    end
    
    % Plot graphs of erros and save into fig files
    h(1) = figure('Visible','off');
    surf(X,Y,Z_e1);
    title(sprintf('Parameter estimation error with fmincon (pob=%.2f, nob=100)',pob));
    l=legend(sprintf('mean(Z) = %d',mean2(Z_e1)));l.Location='best';
    xlabel('X = (h/hmax)')
    ylabel('Y = sin(theta)')
    zlabel('Z = Parameter error');
    axis([0 1 0 1 0 1]);
    % save it
    fname = sprintf(filenameformat,k);
    saveas(h,[pwd fname])
    close(h)
    
    mZ_e1(k) = mean2(Z_e1);
end

f = figure('Visible','off');
plot(pob_c, mZ_e1);
title('Mean parameter estimation error with fmincon');
xlabel('X = Percentage of observed curve')
ylabel('Y = Mean error over parameters')
% axis([0 1 0 1]);
fname = sprintf(filenameformat,k+1);
saveas(f,[pwd fname])
close(f)


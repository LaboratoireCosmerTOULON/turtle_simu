function [ s_gna ] = estimateCatenaryParams( fig, Pcat2d_samp, rlen, hmax, s_init, lb, ub )
% estimateCatenaryParams Estimates the catenary 3D parameters a=h/hmax and
% b = sin(theta) through a Gauss-Newton approximation
% INPUTS:
%   fig         :   figure where plot estimated catenary
%   Pcat2d_samp :   samples of observed rope
%   rlen        :   rope half-length
%   hmax        :   rope maximum sag
%   s_init      :   initial guess for parameters estimation
%   lb, ub      :   parameters lower and upper bounds
% OUTPUTS
%   s_gna       :   estimated catenary parameters s_gna = (a,b)

% handle with signal of parameter b = sin(theta). If b < 0, invert signal
% of Pcat2d_samp x-coordinate since we estimate a 'b' belonging to [0,1]
inv = 0; % flag for signaling b negative
if( mean(Pcat2d_samp(1,:)) < 0 ) % if the rope is in the right of the robot XZ plane, b < 0
    Pcat2d_samp(1,:) = -Pcat2d_samp(1,:); 
    inv = 1;
end

% Fitting with Gauss-Newton method
[s_gna,steps,chisq] = GaussNewton_v2(Pcat2d_samp(1,:),Pcat2d_samp(2,:),rlen,hmax,s_init,lb,ub); % minimization with dampled GNA
% if sin(theta) is negative, invert sign of b
if(inv == 1)
    s_gna(2) = -s_gna(2);
end

% plot estimated catenary for visualization
drawCatenaryEstim( fig, rlen, hmax, s_gna );

end


function [ v_pC, s_gna ] = getVelocity( fig, Pcat2d_samp, rlen, hmax, s_d, s_init, lb, ub )
% getVelocity performs the visual servoing
%   Detailed explanation goes here

% Estimate the catenary parameters with Gauss-Newton Algorithm
[ s_gna ] = estimateCatenaryParams( fig, Pcat2d_samp, rlen, hmax, s_init, lb, ub );

% Calculation of interaction matrix
% First, retrieve 3D coordinates of rope
Pcat3d_gna = catenary3D(rlen,hmax,s_gna);
p_A = Pcat3d_gna(:,end); % coordinates of rope attachment point at robot r1 in rope-frame (p_C)
% Calculate interaction matrix ds = L*v_pC
L = interactionMatrix(rlen,hmax,s_gna, p_A(1), p_A(2), p_A(3));
gain = 1;
s_e = s_gna - s_d;
v_pC = -gain*pinv(L)*s_e;


%
% R_w_r2 = rotMtx(p_w_r2(4:6)); % rotation matrix World-to-robot
% J_w_r2 = rotJacobian(p_w_r2(4:6)); % World-to-robot
% T_r2_w = transf_W2R(R_w_r2',inv(J_w_r2)); % Inverse transformation: R2W

end


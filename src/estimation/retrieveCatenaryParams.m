function [ a, b ] = retrieveCatenaryParams( rlen, hmax, p_w_ropeA, p_w_ropeC, p_w_r2 )
% retrieveCatenaryParams calculates the catenary parameters a = h/hmax and
% b = sin(theta) from the current rope position

% Rope attachment points 
pA_x = p_w_ropeA(1);
pA_y = p_w_ropeA(2);
pC_x = p_w_ropeC(1);
pC_y = p_w_ropeC(2);
    
% Calculation of a = h/hmax
d2 = 0.25*((pA_x - pC_x)^2 + (pA_y - pC_y)^2); % square half-distance between rope attachements points
h = sqrt(rlen^2 - d2); % McLaurin approximation for calculation the rope sag
a = h/hmax;

% Calculation of b = sin(theta)
w_R_r2 = angles2rotMtx(p_w_r2(4:6)); % rotation matrix world-to-r2
r2_R_w = w_R_r2'; % inverse: r2-to-world
alpha = atan2(pA_y-pC_y,pA_x-pC_x); % rope yaw in world-frame
w_R_rope = angles2rotMtx([0; 0; alpha]); % rotation matrix for rope angle in world-frame
r2_R_rope = r2_R_w*w_R_rope; % rope angle in robot frame
angles = rotMtx2angles(r2_R_rope);
b = sin(angles(3)); % b = sin(psi)
end
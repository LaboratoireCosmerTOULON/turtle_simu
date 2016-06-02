function [ P_cam_cat2d_samp ] = draw2dCatenary(fig,p_w_cam,p_w_ropeC,rlen,hmax,p)

% Catenary points in frame pC
P_pC_cat2d = catenary2D(rlen,hmax,p);

% % Transformation: from pC-frame to camera-frame
% R_w_pC = rotMtx(p_w_ropeC(4:6)); % rotation matrix World-to-camera
% M_pC_w = homogeneousTransf(R_w_pC', -p_w_ropeC(1:3));
% R_w_cam = rotMtx(p_w_cam(4:6)); % rotation matrix World-to-camera
% M_w_cam = homogeneousTransf(R_w_cam, p_w_cam(1:3));
% P_cam_cat2d = M_w_cam*M_pC_w*[P_pC_cat2d;1];
% % Camera stenope
% P_cam_cat2d(1,:) = P_cam_cat2d(1,:)./P_cam_cat2d(2,:);
% P_cam_cat2d(2,:) = P_cam_cat2d(2,:)./P_cam_cat2d(2,:);

P_cam_cat2d = P_pC_cat2d;
snr = 50;
pob = 1.0;
P_cam_cat2d_samp = catenarySampling(P_cam_cat2d, pob, snr);


% Plot scene
figure(fig);
subplot(1,2,2);
% plot catenary
plot(P_cam_cat2d_samp(1,:),P_cam_cat2d_samp(2,:),'+');
hold on;

% plot options
% plot options
title('Catenary 2D')
xlabel('y')
ylabel('z')
axis equal
axis([-2*rlen 2*rlen -hmax 0.1])
set(gca,'Xdir','reverse')

end



function [ Pcat2d_gna ] = drawCatenaryEstim( fig, rlen, hmax, p_gna )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

Pcat2d_gna = catenary2D(rlen,hmax,p_gna);
figure(fig);
subplot(1,2,2);
plot(Pcat2d_gna(2,:),Pcat2d_gna(3,:),'-g');

end


function [ body, cap ] = drawTurtle( p_w_r, turtle_radius )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

%[x_cir,y_cir] = % HACK CLAIRE VERSION circle(p_w_r(1),p_w_r(2),turtle_radius);

x_cir = p_w_r(1);
y_cir = p_w_r(2);

x_cap = [p_w_r(1) p_w_r(1)+turtle_radius*cos(p_w_r(6))];
y_cap = [p_w_r(2) p_w_r(2)+turtle_radius*sin(p_w_r(6))];

body = [x_cir; y_cir]; % turtle body points
cap = [x_cap; y_cap]; % turtle cap points

end


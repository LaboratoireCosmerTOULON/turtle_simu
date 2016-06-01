function drawScene( fig, p_w_r1, p_w_r2, turtle_radius, p_w_ropeA, p_w_ropeC )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

[turtle1_body, turtle1_cap] = drawTurtle(p_w_r1, turtle_radius);
[turtle2_body, turtle2_cap] = drawTurtle(p_w_r2, turtle_radius);

% Plot scene
figure(fig);
subplot(1,2,1);
% plot turtle 1
plot(turtle1_body(1,:),turtle1_body(2,:),'b')
hold on
plot(turtle1_cap(1,:),turtle1_cap(2,:),'b')
% plot turtle 2
plot(turtle2_body(1,:),turtle2_body(2,:),'b');
plot(turtle2_cap(1,:),turtle2_cap(2,:),'b')
% plot rope
plot([p_w_ropeC(1) p_w_ropeA(1)],[p_w_ropeC(2) p_w_ropeA(2)],'r');

% plot options
title('Inertial frame')
xlabel('x')
ylabel('y')
axis equal
axis([0 3 0 3])

end


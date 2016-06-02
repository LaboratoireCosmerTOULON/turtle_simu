function [ x, y ] = circle( xo, yo, r )
% circle returns vectos x and y corresponding to a circle of center (xo,yo)
% and radius r
% 0.01 is the angle step, bigger values will draw the circle faster but
% you might notice imperfections (not very smooth)

ang = 0:0.01:2*pi; 
xp = r*cos(ang);
yp = r*sin(ang);
x = xo+xp;
y = yo+yp;

end
function P = catenary3D(rlen,hmax,p)
    % function for calculation of a 3D catenary equation
    % R = rope half-length
    % D = span between attachment points
    % p = catenary parameters : 
    %     p(1) = curve sag 'H'
    %     p(2) = sinus of curve angle with robot azimuth
    % P = [x;y;z] = output matrix with catenary coordinates
    
    a = p(1);
    b = p(2);
    h = a*hmax;
    
    C = 2*h/(rlen^2 - h^2);
    D = (1/C)*acosh(C*h + 1);
    x = linspace(0,2*D*(sqrt(1-b^2)),1001); % 2D*cos(arcsin(sin(theta)))
    y = linspace(0,2*D*b,1001); % 2D*sin(theta)
    t = sqrt(x.^2 + y.^2);
    z = (1/C)*(cosh(C*(t-D))-1)-h; % catenary
    P = [x;y;z];
end
function value = catenaryModelDerivative(xk,r,hmax,Tcam,s,index)
% This function calculates the catenary model partials derivatives (dy/da and
% dy/db);
% Remember: the residual is defined as rk = yk - y(xk,p), where p is the
% vector of parameters p = [a b]. If you wish the residuals
% partial derivatives, please do: drk/dp = -dy(xk,p)/dp;

a   = s(1);
b   = s(2);
h   = a*hmax;
C   = 2*a*hmax/(r^2 - (a*hmax)^2);
D   = (1/C)*acosh(C*a*hmax + 1);
Tx  = Tcam(1);
Ty  = Tcam(2);
Tz  = Tcam(3);

q1 = (b + sqrt(1-b^2)*xk)./(sqrt(1-b^2)*Tx + b*Tz);
q2 = (Tx - Tz*xk)./(sqrt(1-b^2)*xk + b);

% dy/da or dy/db
switch index
    % dy/da, partial with respect to a = h/hmax
    case 1
        diCda   = -(r^2 + h^2)/(2*h*a); % diCda = d(1/C)/da
        AIa     =  -q1*diCda*(cosh(C*q2 - C*D)-1);
        AIb1    =  q2*(C*(r^2 + h^2))/(a*(r^2 - h^2));
        AIb2    = -((r^2 + h^2)/(a*(r^2 - h^2)))*acosh(C*h + 1);
        AIb3    = -(AIb2 + (C*r)^2/(a*sqrt((C*h)^2 + 2*C*h)));
        AIb     = -(q1/C)*sinh(C*q2 - C*D)*(AIb1 + AIb2 + AIb3);
        AI      =  AIa + AIb;
        AII     = hmax*q1;
        value   = (AI + AII);
    % dy/b, partial with respect to b = sin(theta)
    case 2 
        dq1db   =  (Tx - xk*Tz)./(sqrt(1-b^2)*(sqrt(1-b^2)*Tx + b*Tz)^2);
        dq2db   = -((Tx - xk*Tz)*(sqrt(1-b^2) - b*xk))./(sqrt(1-b^2)*(b+sqrt(1-b^2)*xk)^2);
        BIa     = -(1/C)*dq1db*(cosh(C*q2 - C*D)-1);
        BIb     = -q1*dq2db*sinh(C*q2 -C*D);
        BI      =  BIa + BIb;
        BII     =  h*dq1db;
        BIII    =  Ty*dq1db;
        value   = BI + BII + BIII;
end
end

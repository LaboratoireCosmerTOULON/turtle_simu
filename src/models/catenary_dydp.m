function value = catenary_dydp(xk,R,Hmax,p,index)
% This function calculates the model partials derivatives (dy/da and
% dr/db);
% Remember: the residual is defined as rk = yk - y(xk,p), where p is the
% vector of parameters p = [a b]. If you wish the residuals
% partial derivatives, please do: drk/dp = -dy(xk,p)/dp;

a = p(1);
b = p(2);
C = 2*a*Hmax/(R^2 - (a*Hmax)^2);
D = (1/C)*acosh(C*a*Hmax + 1);
switch index
    case 1 % dy/da, partial with respect to a = h/hmax
        % drda = d((1/C)*(cosh(C*x./b - C*D)-1)-a*Hmax)/da
        diCda = -(R^2 + (a*Hmax)^2)/(2*Hmax*a^2); % diCda = d(1/C)/da
        Ia = diCda*(cosh(C*xk/b - C*D)-1);
        Ib1 = (C*xk*(R^2 + (a*Hmax)^2))/(a*b*(R^2 - (a*Hmax)^2));
        Ib2 = -((R^2 + (a*Hmax)^2)*acosh(C*a*Hmax + 1))/(a*(R^2 - (a*Hmax)^2));
        Ib3 = -(Ib2 + (C*R)^2/(a*sqrt((C*a*Hmax)^2 + 2*C*a*Hmax)));
        Ib = (1/C)*sinh(C*xk/b - C*D)*(Ib1 + Ib2 + Ib3);
        I = Ia + Ib;
        II = -Hmax;
        value = (I + II);
    case 2 % dy/b, partial with respect to b = sin(theta)
        value = -(xk/(b^2))*sinh(C*xk/b - C*D);
end
end

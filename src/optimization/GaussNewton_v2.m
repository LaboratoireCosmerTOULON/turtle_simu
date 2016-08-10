function [s_min,steps,chisq] = GaussNewton_v2(xi,yi,rlen,hmax,s_init,lb,ub,pc)
%
% GaussNewton_v2 (dampled and bounded version)
%
% Gauss-Newton with bound constraints to perform a non-linear least square 
% Takes as input a row vector of x and y values, and a column vector of
% initial guesses. Partial derivatives for the jacobian must entered below
% as a function 'derivative_dyds()'
%
% rlen : the rope length
% hmax : the rope maximum sag 

format long
tolChisq= 1e-2; 				% set a value for the accuracy
maxstep = 20; 					% set maximum number of steps to run for
m       = length(xi); 			% determine number of measurements
n       = length(s_init); 		% determine number of parameters
s       = s_init;				% initial guess
s_old   = s;
chisq   = 0;                    % merit function
chisqOld= chisq;
J       = zeros(m,n); 			% create Jacobian matrix
r       = zeros(m,1); 			% vector of residuals

for k = 1:maxstep % iterate through process
    
    chisq = 0; % set merit function to zero for each iteration    
    % ----------------------------------------------------------- %
    %        Calculation of residuals and function Jacobian       %
    % ----------------------------------------------------------- %
    
    % Calculate the model y = f(xi,p)
    Pcat2d = catenaryProjection(rlen,hmax,s,xi,pc);
    y = Pcat2d(2,:);
    J = zeros(m,n);
    r = zeros(m,1); 
    outliers = [];
    for i = 1:m % for all measurements
        % Calculate the sum of squares of residuals
        r(i,1)  = yi(i) - y(i);
        chisq   = chisq + r(i,1)^2;
       
        % Calculation of the Jacobian (dr/ds)
        for j=1:n
            J(i,j) = -catenaryModelDerivative(xi(i),rlen,hmax,pc,s,j); % drds = -dyds
        end
        
        limJ = 50.0;
        if(J(i,1) < -limJ || J(i,2) < -limJ || J(i,1) > limJ || J(i,2) > limJ || isnan(J(i,1)) || isnan(J(i,2)))
            outliers(end + 1) = i;
        end

    end
    % Remove outliers
    J(outliers,:) = [];
    r(outliers,:) = [];
    
    % ----------------------------------------------------------- %
    %            Calculate new approximation
    % ----------------------------------------------------------- %
    
    % Initialization of lambda, the dampling coefficient that avoids the parameters estimation outside the bounds 
    Lbda = [1.0 1.0]; 
    Lambda = diag(Lbda); % built a diagonal matrix of lambdas
    s = s_old - Lambda*pinv(J)*r;
    % Ensure that s is not out of bounds
    while(sum(s < lb) > 0 || sum(s > ub) >0) % while P is out of bounds, reduce lambda by 10
        for i=1:n
            if(s(i) < lb(i) || s(i) > ub(i))
                Lbda(i) = 0.1*Lbda(i);
            end 
        end
        Lambda = diag(Lbda);
        s = s_old - Lambda*pinv(J)*r; 
    end

    % For while, the current vector s is the minimizer of objective
    % function
    s_min = s;
    % Calculate estimation improvement and break loop is improvement is
    % less than tolerance
    deltaChisq = abs(chisq - chisqOld);
    if (deltaChisq <= tolChisq);
        break
    end
    % Otherwise continue loop
    s_old = s; %set s to s_old
    chisqOld = chisq;
end
steps = k;
end

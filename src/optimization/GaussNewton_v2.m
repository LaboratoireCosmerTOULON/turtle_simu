function [Pmin,steps,chisq] = GaussNewton_v2(xi,yi,rlen,hmax,pinit,lb,ub,pc)
%
% GaussNewton_v2 (dampled and bounded version)
%
% Gauss-Newton with bound constraints to perform a non-linear least square 
% Takes as input a row vector of x and y values, and a column vector of
% initial guesses. Partial derivatives for the jacobian must entered below
% as a function 'derivative_dydp()'
%
% rlen : the rope length
% hmax : the rope maximum sag 

format long
tol     = 1e-8; 				% set a value for the accuracy
maxstep = 30; 					% set maximum number of steps to run for
m       = length(xi); 			% determine number of measurements
n       = length(pinit); 		% determine number of parameters
P       = pinit;				% initial guess
Pold   = P;
J       = zeros(m,n); 			% create Jacobian matrix
chisq   = 0; 					% set the merit function chisq to zero
R       = zeros(m,1); 			% vector of residuals
DeltaP  = zeros(maxstep,1); 	% vector containing the value p-p_old



for k = 1:maxstep % iterate through process
    %disp(Pold);
    
    chisq = 0; % set merit function to zero for each iteration
    
    % ----------------------------------------------------------- %
    %        Calculation of residuals and function Jacobian       %
    % ----------------------------------------------------------- %
       
    Pcat2d = catenaryProjection(rlen,hmax,P,xi,pc); % calculate the model y = f(xi,p)
    y = Pcat2d(2,:);
    for i = 1:m % for all measurements
%         fprintf('i: %d size(J): %d %d \n',i,size(J));
        R(i,1) = yi(i) - y(i);
        chisq = chisq + R(i,1)^2; % calculate the sum of squares of residuals
       
        % Calculation of the Jacobian (dr/dp)
        for j=1:n % for all parameters
            J(i,j) = -catenaryModelDerivative(xi(i),rlen,hmax,pc,P,j); % drdp = -dydp
        end
        if(J(i,1) < -100.0 || J(i,2) < -100.0 || J(i,1) > 100.0 || J(i,2) > 100.0 || isnan(J(i,1)) || isnan(J(i,2)))
            J(i,:) = 0;
%             R(i,:) = [];  
        end
%         fprintf('J(%d,:)= %.6f %.6f \n',i,J(i,:));
    end
%     fprintf('mean(J): %.6f, %.6f \n',mean(J));
%     fprintf('max(J): %.6f, %.6f \n',max(J));
%     fprintf('max(J): %.6f, %.6f \n',min(J));
    
    
    % ----------------------------------------------------------- %
    %            Calculate new approximation
    % ----------------------------------------------------------- %
    
    Lbda = [1.0 1.0]; % initial value for lambda, the dampling coefficient that avoids the parameters estimation outside the bounds 
    Lambda = diag(Lbda); % built a diagonal matrix of lambdas
    P = Pold - Lambda*pinv(J)*R;
    % Ensure that P is not out of bounds
    while(sum(P < lb) > 0 || sum(P > ub) >0) % while P is out of bounds, reduce lambda by 10
        for i=1:n
            if(P(i) < lb(i) || P(i) > ub(i))
                Lbda(i) = 0.1*Lbda(i);
            end 
        end
        Lambda = diag(Lbda);
        P = Pold - Lambda*pinv(J)*R; 
    end
%     fprintf('%d: p_old = [%.4f %.4f], Lbda = (%.4f %.4f), deltap = [%.4f %.4f]\n',k,Pold,Lbda,pinv(J)*R);
    Pmin = P;
    
    
    DeltaP(k) = (1/sqrt(2))*sqrt((P(1) - Pold(1))^2 + (P(2) - Pold(2))^2); % calculate estimation improvement
    
    if (DeltaP(k) <= tol); %if less than tolerance break
        break
    end
    Pold = P; %set p to pold
end
steps = k;
end

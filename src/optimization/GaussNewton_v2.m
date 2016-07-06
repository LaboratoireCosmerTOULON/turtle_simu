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
tol     = 1e-8; 				% set a value for the accuracy
maxstep = 20; 					% set maximum number of steps to run for
m       = length(xi); 			% determine number of measurements
n       = length(s_init); 		% determine number of parameters
s       = s_init;				% initial guess
s_old   = s;
J       = zeros(m,n); 			% create Jacobian matrix
r       = zeros(m,1); 			% vector of residuals
delta_s = zeros(maxstep,1); 	% vector containing the value s-s_old

for k = 1:maxstep % iterate through process
    
    chisq = 0; % set merit function to zero for each iteration    
    % ----------------------------------------------------------- %
    %        Calculation of residuals and function Jacobian       %
    % ----------------------------------------------------------- %
    
    % Calculate the model y = f(xi,p)
    Pcat2d = catenaryProjection(rlen,hmax,s,xi,pc);
    y = Pcat2d(2,:);
    outliers = [];
    for i = 1:m % for all measurements
%         fprintf('i: %d size(J): %d %d \n',i,size(J));
        % Calculate the sum of squares of residuals
        r(i,1)  = yi(i) - y(i);
        chisq   = chisq + r(i,1)^2;
       
        % Calculation of the Jacobian (dr/ds)
        for j=1:n
            J(i,j) = -catenaryModelDerivative(xi(i),rlen,hmax,pc,s,j); % drds = -dyds
        end
        
%         dh = 10^-6;
%         Fah = catenaryProjection(rlen,hmax,[s(1)+dh;s(2)],xi(i),pc);
%         Fa  = catenaryProjection(rlen,hmax,s,xi(i),pc);
%         yah = Fah(2);
%         ya  = Fa(2);
%         Fbh = catenaryProjection(rlen,hmax,[s(1);s(2)+dh],xi(i),pc);
%         Fb  = catenaryProjection(rlen,hmax,s,xi(i),pc);
%         ybh = Fbh(2);
%         yb  = Fb(2);
%         drda   = -(yah - ya)/dh;
%         drdb   = -(ybh - yb)/dh;
% %         fprintf('i%d : Ja= %.6f %.6f Jn= %.6f %.6f \n',i,J(i,:),drda,drdb);
%         J(i,1) = drda;
%         J(i,2) = drdb;
        
        limJ = 50.0;
        if(J(i,1) < -limJ || J(i,2) < -limJ || J(i,1) > limJ || J(i,2) > limJ || isnan(J(i,1)) || isnan(J(i,2)))
            outliers(end + 1) = i;
        end

    end
    
    % Remove outliers
    J(outliers,:) = [];
    r(outliers,:) = [];
    
%     for i=1:length(r)
%         fprintf('i%d : J= %.6f %.6f, r= %.6f \n',i,J(i,:),r(i,:));
%     end
    
%     fprintf('mean(J): %.6f, %.6f \n',mean(J));
%     fprintf('max(J): %.6f, %.6f \n',max(J));
%     fprintf('max(J): %.6f, %.6f \n',min(J));
    
    
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
%     fprintf('%d: s_old = [%.4f %.4f], Lbda = (%.4f %.4f), ds = [%.4f %.4f]\n',k,s_old,Lbda,pinv(J)*r);
    
    % For while, the current vector s is the minimizer of objective
    % function
    s_min = s;
    % Calculate estimation improvement and break loop is improvement is
    % less than tolerance
    delta_s(k) = (1/sqrt(2))*sqrt((s(1) - s_old(1))^2 + (s(2) - s_old(2))^2);
    if (delta_s(k) <= tol);
        break
    end
    % Otherwise continue loop
    s_old = s; %set s to s_old
end
steps = k;
end

function soma = catenaryObjectiveFunc(xi,yi,R,Hmax,p)
    P = catenary2D(R,Hmax,p,xi);
	y = P(3,:);
    soma = sum((yi-y(1:length(yi))).^2);
    
end

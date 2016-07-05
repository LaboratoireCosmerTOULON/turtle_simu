function soma = fcout1(xi,yi,R,Hmax,s,pc)
    P = catenaryProjection_vincent(R,Hmax,s,xi,pc);
	y = P(2,:);
    soma = sum((yi-y(1:length(yi))).^2);    
end
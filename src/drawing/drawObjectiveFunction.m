function drawObjectiveFunction(xi,yi,R,Hmax,pc,index)
switch index
    case 1
        [X,Y] = meshgrid(0:0.01:1,0:0.01:1);
        Z = zeros(size(X));
        sz = size(X);
        for i=1:sz(1)
            for j = 1:sz(2)
                param = [X(1,j), Y(i,1)];
                y = catenaryProjection(R,Hmax,param,xi,pc);
                Z(i,j) = sum((yi-y(1:length(yi))).^2);
            end
        end
        
        figure();
        plt = surf(X,Y,Z);
        l=legend('Fitting square error');l.Location='best';
        title('Classic cost function');
        xlabel('X = (h/hmax)')
        ylabel('Y = sin(theta)')
        zlabel('Z = cost');
        colorbar;
        caxis([0,100])
        axis([0 1 0 1 0 100]);
        set(plt, 'edgecolor','none');

    case 2
        [X,Y] = meshgrid(-10:0.05:10,-10:0.05:10);
        Z = zeros(size(X));
        sz = size(X);
        for i=1:sz(1)
            for j = 1:sz(2)
                param = [X(1,j), Y(i,1)];
                y = catenaryProjection(R,Hmax,param,xi,pc);
                Z(i,j) = sum((yi-y(1:length(yi))).^2);
            end
        end
        
        figure();
        plt = surf(X,Y,Z);
        l=legend('Fitting square error');l.Location='best';
        title('Classic cost function');
        xlabel('X = (h/hmax)')
        ylabel('Y = sin(theta)')
        zlabel('Z = cost');
        colorbar;
        caxis([0,10000])
        axis([0 1 0 1 0 10000]);
        set(plt, 'edgecolor','none');
end
end
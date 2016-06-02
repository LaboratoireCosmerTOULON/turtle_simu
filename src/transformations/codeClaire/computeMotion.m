function [ M ] = computeMotion(v,dt)
 if(length(v)==6)
    M = expMapDirectRxRyRz(v,dt);
 else 
    M = eye(4,4);
    disp('WARNING <<computeMotionCam: v = 3');
 end;
end

function [ vitesse ] = expMapInverseRxRyRz( M, dt )
  
vitesse = expMapInverseThetaU(M,dt);

end

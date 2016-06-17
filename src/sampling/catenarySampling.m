function [ Pcat_samp ] = catenarySampling( Pcat2d, pob, snr )
% generateCatenary generate sampling points from the catenary points 'Pcat'
% Only a part 'pob' of the catenary is observed (pob belogns to [0-1]);
% some noise is added to the observation (snr = signal noise ratio)

% Simulate a noisy observation
x_noi = awgn(Pcat2d(1,:), snr);
y_noi = awgn(Pcat2d(2,:), snr);
% Observation
nob = 100; % number of samples
x_ob = x_noi(1:floor(pob*length(x_noi)/nob):floor(pob*length(x_noi)));
y_ob = y_noi(1:floor(pob*length(y_noi)/nob):floor(pob*length(y_noi)));

Pcat_samp = [x_ob; y_ob];

end


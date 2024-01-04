function observations = getObservations(trueTracks,parameters)
numSteps = parameters.numSteps;
sigmaMeasurementNoise = parameters.sigmaMeasurementNoise;

% load measurement matrix
[~,~,H] = getModelMatrices(0);

%ground truth plus some Gaussian noise
observations = H*trueTracks + sigmaMeasurementNoise*randn(2,numSteps);

end


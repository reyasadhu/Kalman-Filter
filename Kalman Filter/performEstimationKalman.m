function [estimatedTrack,innovationSequence] = performEstimationKalman(observations, parameters)

% load system parameters
numSteps = parameters.numSteps;
scanTime = parameters.scanTime;
sigmaDrivingNoise = parameters.sigmaDrivingNoise;
sigmaMeasurementNoise = parameters.sigmaMeasurementNoise;
priorMean = parameters.priorMean;
priorCovariance = parameters.priorCovariance;

[A,W,H] = getModelMatrices(scanTime);
drivingNoiseCovariance = diag([sigmaDrivingNoise^2;sigmaDrivingNoise^2]); 
measurementNoiseCovariance = diag([sigmaMeasurementNoise^2;sigmaMeasurementNoise^2]); 

% initialize estimated means and covariances
estimatedMeans = zeros(4,numSteps+1);
estimatedCovariances = zeros(4,4,numSteps+1);
estimatedMeans(:,1) = priorMean;
estimatedCovariances(:,:,1) = priorCovariance;
innovationSequence = zeros(2,numSteps);


for step = 1:numSteps
  %todo
  prevcov=A*estimatedCovariances(:,:,step)*A'+W*drivingNoiseCovariance*W';
  prevmean=A*estimatedMeans(:,step);
  innovationSequence(:,step)=(observations(:,step)-H*prevmean);
  Kgain=prevcov*H'/(H*prevcov*H'+measurementNoiseCovariance);
  estimatedMeans(:,step+1)= prevmean+Kgain*innovationSequence(:,step);
  estimatedCovariances(:,:,step+1)=prevcov-Kgain*H*prevcov;
 
end

% remove prior information from estimated track
estimatedTrack = estimatedMeans(:,2:numSteps+1);
end


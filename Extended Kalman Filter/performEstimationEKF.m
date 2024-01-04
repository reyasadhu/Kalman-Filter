function estimatedTrack = performEstimationEKF(observations, parameters)

% load system parameters
numSteps = parameters.numSteps;
scanTime = parameters.scanTime;
sigmaDrivingNoise = parameters.sigmaDrivingNoise;
sigmaMeasurementNoiseRange = parameters.sigmaMeasurementNoiseRange;
sigmaMeasurementNoiseBearing = parameters.sigmaMeasurementNoiseBearing;
sensorPosition = parameters.sensorPosition;
priorMean = parameters.priorMean;
priorCovariance = parameters.priorCovariance;

[A,W,~] = getModelMatrices(scanTime);
drivingNoiseCovariance = diag([sigmaDrivingNoise^2;sigmaDrivingNoise^2]); 
measurementNoiseCovariance = diag([sigmaMeasurementNoiseRange^2;sigmaMeasurementNoiseBearing^2]); 

% initialize estimated means and covariances
estimatedMeans = zeros(4,numSteps+1);
estimatedCovariances = zeros(4,4,numSteps+1);
estimatedMeans(:,1) = priorMean;
estimatedCovariances(:,:,1) = priorCovariance;


for step = 1:numSteps

  
  %todo: Kalman prediction step
  prevcov=A*estimatedCovariances(:,:,step)*A'+W*drivingNoiseCovariance*W';
  prevmean=A*estimatedMeans(:,step);
  

  %todo: implement EKF update step
  H=Jacobian(prevmean,sensorPosition);
  innovationSequence=(observations(:,step)-calculateh(prevmean,sensorPosition));
  Kgain=prevcov*H'/(H*prevcov*H'+measurementNoiseCovariance);
  estimatedMeans(:,step+1)= prevmean+Kgain*innovationSequence;
  estimatedCovariances(:,:,step+1)=prevcov-Kgain*H*prevcov;

  estimatedCovariances(:,:,step+1) = checkAndFixCovarianceMatrix( estimatedCovariances(:,:,step+1), 10^(-10) );
end

% remove prior information from estimated track
estimatedTrack = estimatedMeans(:,2:numSteps+1);
end

function output=Jacobian(x,p)
    output=zeros(2,4);
    norm=((x(1)-p(1))^2+(x(2)-p(2))^2);
    output(1,1)=(x(1)-p(1))/sqrt(norm);
    output(1,2)=(x(2)-p(2))/sqrt(norm);
    output(2,1)=(x(2)-p(2))/norm;
    output(2,2)=-(x(1)-p(1))/norm;
end

function output=calculateh(x,p)
    output=zeros(2,1);
    output(1)=sqrt((x(1)-p(1))^2+(x(2)-p(2))^2);
    output(2)=wrapTo2Pi(atan2(x(1)-p(1),x(2)-p(2)));
    
end
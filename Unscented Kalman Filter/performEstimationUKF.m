function estimatedTrack = performEstimationUKF(observations, parameters)

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
  [sp,weights]=getSigmaPoints(estimatedMeans(:,step),estimatedCovariances(:,:,step));
  new_sp=A*sp;
  prevmean=sum((weights'.*new_sp),2);
  prevcov=weights'.*(new_sp-prevmean)*(new_sp-prevmean)'+W*drivingNoiseCovariance*W';
  
  
  %todo: implement UKF update step
  
  sp=new_sp;
  [L,~]=size(sp);
  y_sp=zeros(2,(2*L+1));
  % calculate sigma points z_(i)=h(x_(i))
  for i=1:2*L+1
      y_sp(:,i)=calculateh(sp(:,i),sensorPosition);
  end

  meany=sum((weights'.*y_sp),2);
  covy=weights'.*(y_sp-meany)*(y_sp-meany)'+measurementNoiseCovariance;
  cov_xy=weights'.*(sp-prevmean)*(y_sp-meany)';
  
  K=cov_xy/covy;
  estimatedMeans(:,step+1)=prevmean+K*(observations(:,step)-meany);
  estimatedCovariances(:,:,step+1)=prevcov-K*cov_xy';
  estimatedCovariances(:,:,step+1) = checkAndFixCovarianceMatrix( estimatedCovariances(:,:,step+1), 10^(-10) );
end

% remove prior information from estimated track
estimatedTrack = estimatedMeans(:,2:numSteps+1);
end

function output=calculateh(x,p)
    output=zeros(2,1);
    output(1)=sqrt((x(1)-p(1))^2+(x(2)-p(2))^2);
    output(2)=wrapTo2Pi(atan2(x(1)-p(1),x(2)-p(2)));
    
end
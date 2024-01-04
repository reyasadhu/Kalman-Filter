function [points, weights] = getSigmaPoints(mean, covariance)
[lengthState, ~] =  size(mean);
numSigmaPoints = 2*lengthState+1;

%determine sigma point weights
lambda = 0;
weights = 1/(2*(lengthState + lambda))*ones(numSigmaPoints,1);
weights(1) = lambda/(lengthState + lambda);

%calculate sigma points
sqrtmCovariance = sqrtm(covariance);
points = zeros(lengthState,2*lengthState+1);
points(:,1) = mean;
points(:,2:lengthState+1) = repmat(mean,1,lengthState) + sqrt(lengthState + lambda)*sqrtmCovariance;
points(:,lengthState+2:2*lengthState+1) = repmat(mean,1,lengthState) - sqrt(lengthState + lambda)*sqrtmCovariance;
end
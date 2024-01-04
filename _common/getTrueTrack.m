function [trueTrack,priorMean] = getTrueTrack(parameters,startState)
numSteps = parameters.numSteps;
scanTime = parameters.scanTime;
sigmaDrivingNoise = parameters.sigmaDrivingNoise;
priorCovariance = parameters.priorCovariance;

% load state-transition matrices
[A,W,~] = getModelMatrices(scanTime);

% initialize true track with start state
trueTrack = zeros(4,numSteps+1);
trueTrack(:,1) = startState;

% evaluate state-transition model for each time step
for step = 1:numSteps
    trueTrack(:,step+1) = A*trueTrack(:,step) + W*sigmaDrivingNoise*randn(2,1);
end
trueTrack = trueTrack(:,2:numSteps+1);

%initialize prior mean
priorMean = startState + sqrt(priorCovariance)*randn(4,1);

end



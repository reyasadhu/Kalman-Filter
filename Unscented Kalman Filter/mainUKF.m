% ECE275A Homework 4, Florian Meyer, 2023

clc; clear variables; close all; addpath('../_common')

rng(1); %generate always the same sequence of ``random'' numbers; change value or comment out for different sequence

% system parameters
parameters.numSteps = 1500;
parameters.scanTime = .1;            

parameters.sigmaDrivingNoise = .05;
parameters.sigmaMeasurementNoiseRange = 5;
parameters.sigmaMeasurementNoiseBearing = .03;
parameters.sensorPosition = [0;50];

parameters.priorCovariance = diag([100;100;20;20]);


% generate true track
startState = [0;0;1;1];
[trueTracks,parameters.priorMean] = getTrueTrack(parameters,startState);

% generate observations
observations = getObservations(trueTracks,parameters);

% perform estimation
estimatedTracks = performEstimationUKF(observations,parameters);

% calculate RMSE
rmse = getError(trueTracks,estimatedTracks);

% plot true and estimated track
figure(1)
plot(trueTracks(1,:),trueTracks(2,:),'LineWidth',1.5)
axis([-50 200 -50 250])
hold on 
plot(estimatedTracks(1,:),estimatedTracks(2,:),'LineWidth',1.5)
scatter(parameters.sensorPosition(1) + observations(1,:).*sin(observations(2,:)),parameters.sensorPosition(2) + observations(1,:).*cos(observations(2,:)),5,'.')
mean(rmse)
legend({'True Track','Estimated Track'},'FontSize',14)
xlabel('x-Coordinate') 
ylabel('y-Coordinate') 


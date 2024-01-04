% ECE275A Homework 4, Florian Meyer, 2023

clc; clear variables; close all; addpath('../_common')

rng(1); %generate always the same sequence of ``random'' numbers; change value or comment out for different sequence

% system parameters
parameters.numSteps = 1500;
parameters.scanTime = .1;             

parameters.sigmaDrivingNoise = .05;    
parameters.sigmaMeasurementNoise = 5;

parameters.priorCovariance = diag([100;100;20;20]);


% generate true track
startState = [0;0;1;1];
[trueTracks,parameters.priorMean] = getTrueTrack(parameters,startState);

% generate observations
observations = getObservations(trueTracks,parameters);

% perform estimation
%(change this to generate model mismatch)
%parameters.sigmaDrivingNoise = 0.000000001;         
%parameters.sigmaMeasurementNoise = 20; 
[estimatedTracks,innovationSequence] = performEstimationKalman(observations,parameters);

% calculate RMSE
rmse = getError(trueTracks,estimatedTracks);

% plot true and estimated track
figure(1)
plot(trueTracks(1,:),trueTracks(2,:),'LineWidth',1.5)
%axis([-50 150 -50 150])
hold on 
plot(estimatedTracks(1,:),estimatedTracks(2,:),'LineWidth',1.5)
scatter(observations(1,:),observations(2,:),5,'.')
mean(rmse)
legend({'True Track','Estimated Track'},'FontSize',14)
xlabel('x-Coordinate') 
ylabel('y-Coordinate') 

% plot innovation sequence
figure(2)
plot(1:parameters.numSteps,innovationSequence(1,:))
hold on
plot(1:parameters.numSteps,innovationSequence(2,:))
xlabel('Time Step') 
ylabel('Innovation Value') 

legend({'First Element','Second Element'},'FontSize',14)

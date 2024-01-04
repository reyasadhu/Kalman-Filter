function observations = getObservations(trueTracks,parameters)
numSteps = parameters.numSteps;
sigmaMeasurementNoiseRange = parameters.sigmaMeasurementNoiseRange;
sigmaMeasurementNoiseBearing = parameters.sigmaMeasurementNoiseBearing;
sensorPosition = parameters.sensorPosition;

observations = observationModel(trueTracks,sensorPosition);
observations = observations + [sigmaMeasurementNoiseRange*randn(1,numSteps); sigmaMeasurementNoiseBearing*randn(1,numSteps)];

end


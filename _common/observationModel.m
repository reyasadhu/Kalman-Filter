function observations = observationModel(objectState,sensorPosition)
numSteps = size(objectState,2);

observations = zeros(2,numSteps);
observations(1,:) = sqrt((objectState(1,:) - sensorPosition(1)).^2+(objectState(2,:) - sensorPosition(2)).^2);
observations(2,:) =  atan2(objectState(1,:) - sensorPosition(1), objectState(2,:) - sensorPosition(2));

end
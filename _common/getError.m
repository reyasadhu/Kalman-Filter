function rmse = getError(trueTracks,estimatedTracks)
numSteps = size(trueTracks,2);
rmse = zeros(1,numSteps);

for step = 1:numSteps
    rmse(step) = sqrt((trueTracks(1,step)-estimatedTracks(1,step))^2+(trueTracks(2,step)-estimatedTracks(2,step))^2);
end

end
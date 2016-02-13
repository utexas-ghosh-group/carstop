clear;
% predict the next maneuver of the vehicle [left, left lane change,
% straight, right lane change, right]
% 2/2/16

load('segment_s2nb2_road.mat');
nobs = size(vehicleMatrix,1);

todo = 1:nobs;
currentPoint = 71;
nBefore = 70;
nAfter = 70;
trainPredictor = @(pathBefore, pathAfter, context) ang2vec(path);

beforeRange = currentPoint-nBefore : currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;
truth = timeMatrix(todo, afterRange, :);
prediction = zeros(length(todo), length(afterRange));

for veh = todo
    pathBefore = timeMatrix(veh, beforeRange, :);
    %pathAfter = timeMatrix(veh, afterRange, :);
    
    thisContext = context(veh, :);
    
    prediction(veh,:) = predict(pathBefore, context);
end

pathsBefore = timeMatrix(todo, beforeRange, :);
predictor = trainPredictor(pathsBefore, pathsAfter, context);
save('predictor.mat','predictor')
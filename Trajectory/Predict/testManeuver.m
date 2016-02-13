clear;
%addpath('/home/motro/Documents/Projects/CarStop/Trajectory/Plot')
% predict the next maneuver of the vehicle [left, left lane change,
% straight, right lane change, right]
% compare to gathered truth
% 2/2/16

load('segment_s2nb2_road.mat');
nobs = size(vehicleMatrix,1);

todo = 1:nobs;
currentPoint = 71;
nBefore = 70;
nAfter = 70;
predictInfo = load('predictor');
predict = @(path, context) CV_prediction(path);
context = roadMatrix;

beforeRange = currentPoint-nBefore : currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;
truth = timeMatrix(todo, afterRange, :);
prediction = zeros(size(truth));

for veh = todo
    pathBefore = squeeze(timeMatrix(veh, beforeRange, :));
    %pathAfter = timeMatrix(veh, afterRange, :);
    
    thisContext = context(veh, :);
    
    prediction(veh,:,:) = predict(pathBefore, context);
end

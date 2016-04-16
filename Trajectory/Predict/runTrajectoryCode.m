% 4/14/16
% handles methods to predict future position
%%
clear;
segmentName = 's3sb3_roadCoord';
load(cat(2,'data_',segmentName,'.mat'));
nobs = size(timeMatrix,1);

% choose the prediction method here
methodName = 'CVroad';
trainMethod = @(in1,in2,in3) CV_train(in1,in2,in3);
testMethod = @(in1,in2,in3,in4) predictKF2(in1,in2,in3,in4,'CVind');
convertToRaw = 1; % run roadCoords2rawCoords.m on your result

% set the observations to train and test on (ex. 1:nobs/2)
% empty train observations [] means the predictor will be loaded from file
% empty test observations [] means predictor will be saved but not used
toTrain = [];
toTest = 1:nobs;

toScore = 0; % 1 yes 0 no



%%
dataTimeMatrix = timeMatrix;
if isempty(contextMatrix)
    contextMatrix = zeros(nobs,1);
end
dataContextMatrix = contextMatrix;
load(cat(2,'truth_',segmentName,'.mat'));
truth = timeMatrix;
clear timeMatrix;

trainTime = dataTimeMatrix(toTrain,:,:);
testTime = dataTimeMatrix(toTest,:,:);
trainDataContext = dataContextMatrix(toTrain,:);
testDataContext = dataContextMatrix(toTest,:);
trainTruth = truth(toTrain,:,:);
testTruth = truth(toTest,:,:);

if ~isempty(toTrain)
    predictor = trainMethod(trainTime, trainDataContext, trainTruth);
    save(cat(2,'predictor_',segmentName,'_',methodName,'.mat'), 'predictor');
else
    try
        load(cat(2,'predictor_',segmentName,'_',methodName,'.mat'));
    catch
        predictor = []; % some methods don't require training (KF, etc.)
    end
end

if ~isempty(toTest)
    predictedTrajectory = testMethod(predictor, testTime, ...
                                testDataContext, (1:size(testTruth,2))/10);
    if convertToRaw
        predictedTrajectory = roadCoords2rawCoords(cat(3,...
                                   predictedTrajectory, testTruth(:,:,3)));
    end
    save(cat(2,'trajectory_',segmentName,'_',methodName,'.mat'), ...
                                'predictedTrajectory');
end

if toScore % confusion matrix thingy
   if isempty(toTest)
       load(cat(2,'trajectory_',segmentName,'_',methodName,'.mat'));
   end
end
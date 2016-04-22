% 4/18/16
% handles methods to predict future position
%%
clear;
segmentName = 'rand1k_roadCoord';
load(cat(2,'data_',segmentName,'.mat'));
nobs = size(timeMatrix,1);

% choose the prediction method here
methodName = 'CVroad';
trainMethod = @(in1,in2,in3) [];%CV_train(in1,in2,in3);
testMethod = @(in1,in2,in3,in4) predictKF(in1,in2,in3,in4,'CVindRoad');
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
        addpath('../Map Code');
        reconvMatrix = cat(2, testTime, ...
                        cat(3, predictedTrajectory, testTruth(:,:,3)));
        predictedTrajectory = roadCoords2rawCoords(reconvMatrix);
        testedTrajectories = size(testTime,2) + 1 :...
                             size(testTime,2) + size(testTruth,2);
        predictedTrajectory = predictedTrajectory(:,testedTrajectories,:);
    end
    save(cat(2,'trajectory_',segmentName,'_',methodName,'.mat'), ...
                              'predictedTrajectory','includedFromSegment');
end

if toScore
   if isempty(toTest)
       load(cat(2,'trajectory_',segmentName,'_',methodName,'.mat'));
   end
   norm2d = @(x, ran) ( x(:,ran,1).^2 + x(:,ran,2).^2 ).^.5;
   if convertToRaw
       oldincluded = includedFromSegment;
       load('truth_s3sb3_raw.mat','timeMatrix');
       includedFromSegment = oldincluded(includedFromSegment>0);
       truth = timeMatrix(includedFromSegment>0,:,1:2);
   else
       truth = testTruth(:,:,1:2);
   end
   Mean_Prediction_Error = mean(mean(norm2d(...
                                 truth - predictedTrajectory, 1:30)))
end
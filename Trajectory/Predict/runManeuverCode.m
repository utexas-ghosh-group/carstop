% 4/11/16
% predicts maneuver from preset categories :
%%
clear;
segmentName = 's3sb2_roadCoord';
load(cat(2,'data_',segmentName,'.mat'));
nobs = size(timeMatrix,1);

trainMethod = @(in1,in2,in3) CV_train(in1,in2,in3);
testMethod = @(in1,in2,in3) predictManeuver_KF(in1,in2,in3);
methodName = 'CV';

% set the observations to train and test on (ex. 1:nobs/2)
% empty train observations [] means the predictor will be loaded from file
% empty test observations [] means predictor will be saved but not used
toTrain = [];
toTest = 1:nobs;

toScore = 1; % 1 yes 0 no


%%
dataTimeMatrix = timeMatrix;
if isempty(contextMatrix)
    contextMatrix = zeros(nobs,1);
end
dataContextMatrix = contextMatrix;
load(cat(2,'truth_',segmentName,'.mat'));
%truthTimeMatrix = timeMatrix;
if isempty(contextMatrix)
    contextMatrix = zeros(nobs,1);
end
truth = contextMatrix;
clear timeMatrix;

trainTime = dataTimeMatrix(toTrain,:,:);
testTime = dataTimeMatrix(toTest,:,:);
trainDataContext = dataContextMatrix(toTrain,:);
testDataContext = dataContextMatrix(toTest,:);
trainTruth = truth(toTrain,:);
testTruth = truth(toTest,:);

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
    predictedManeuvers = ...
                    testMethod(predictor, testTime, testDataContext);
    save(cat(2,'maneuvers_',segmentName,'_',methodName,'.mat'), ...
                                'predictedManeuvers');
end

if toScore % confusion matrix thingy
   if isempty(toTest)
       load(cat(2,'maneuvers_',segmentName,'_',methodName,'.mat'));
   end
   
   confusionMatrix = zeros(6,6);
   for obs = 1:nobs
       trueManeuver = testTruth(obs,1) + 1;
       predictedManeuver = predictedManeuvers(obs)+1;
       if trueManeuver > 0 && predictedManeuver > 0
           confusionMatrix(trueManeuver, predictedManeuver) =...
                    confusionMatrix(trueManeuver, predictedManeuver) + 1;
       end
   end
   confusionMatrix
   accuracy = trace(confusionMatrix)/nobs
end
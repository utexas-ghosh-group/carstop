clear;
load('segment_s2nb_2.mat');
%load('truth_s3sb_3.mat');
%load newtruth4now.mat
load truth_s2nb_2.mat
nobs = size(vehicleMatrix,1);

todo = 1:nobs;%randi(nobs,1,300); todo = DiscreteHist(todo)';
currentPoint = 71;
nBefore = 50; % the number of timesteps to compare to the centroids
nToPredict = 50;

truth = timeMatrix(:,currentPoint+1 : currentPoint+nToPredict, :);
prediction = zeros(size(truth));

%MPs = struct('premean',[],'prevar',[],'aftmean',[],'aftvar',[]);
MPs = struct('weight',[]);
% construct ideal motion patterns
labels = DiscreteHist(nextTruth);
for ii = 1:length(labels)
    allObs = find(nextTruth == labels(ii));
    pathsBefore = timeMatrix(allObs, ...
        currentPoint-nBefore+1 : currentPoint,:);
    pathsAfter = timeMatrix(allObs, ...
        currentPoint+1 : currentPoint+nToPredict,:);
    %MPs(ii).premean = squeeze(mean(pathsBefore,1));
    %MPs(ii).prevar = squeeze(std(pathsBefore,1));
    %MPs(ii).aftmean = squeeze(mean(pathsAfter,1));
    %MPs(ii).aftvar = squeeze(std(pathsAfter,1));
    MPs(ii).weight = regressMP(pathsBefore,pathsAfter, []);
end

for obs = todo
    path = squeeze(timeMatrix(obs,currentPoint-nBefore+1 : currentPoint,:));
    MP = MPs(labels == nextTruth(obs));
    %prediction(obs,:,:) = predictHH(path, MP.aftmean, nToPredict);
    mpPath = regressMP(path, [], MP.weight);
    prediction(obs,:,:) = predictHH(path, mpPath, nToPredict);
end
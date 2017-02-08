clear;
% Takes segment_XXX.mat files and removes all but position coordinates
% gets direction labels
% 4/26/16 not done yet
%%
segmentString = 'rand2k';
%%
load(cat(2,'segment_',segmentString,'.mat'));

importUsefulFunctions();

nobs = size(timeMatrix,1);

todo = 1:nobs;
currentPoint = 71;
nBefore = 70;
nAfter = 50;
beforeRange = currentPoint-nBefore:currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;

dataTimeMatrix = zeros(length(todo),nBefore+1,2);
truthTimeMatrix = zeros(length(todo),nAfter,2);
includedFromSegment = ones(length(todo),1);
directionLabels = zeros(length(todo),1);
for obsidx = 1:length(todo)
    obs = todo(obsidx);
    
    roadsBefore = squeeze(timeMatrix(obs, beforeRange, 5));
    dataTimeMatrix(obsidx,:,:) = timeMatrix(obs, beforeRange, 1:2);
    
    roadsAfter = squeeze(timeMatrix(obs, afterRange, 5));
    truthTimeMatrix(obsidx,:,:) = timeMatrix(obs, afterRange, 1:2);
    
    % maneuver labelling
    if any(roadsAfter == 0) || any(roadsBefore == 0)
        includedFromSegment(obsidx) = 0;
    else
        directionLabels(obsidx) = labelDirection(roadsBefore, roadsAfter);
    end
end

roadMatrix = squeeze(timeMatrix(todo,:,5));
dataTimeMatrix = dataTimeMatrix(includedFromSegment>0,:,:);
truthTimeMatrix = truthTimeMatrix(includedFromSegment>0,:,:);
roadMatrix = roadMatrix(includedFromSegment>0,:);
directionLabels = directionLabels(includedFromSegment>0);

%% save
timeMatrix = cat(3, dataTimeMatrix, roadMatrix(:,beforeRange));
contextMatrix = [];
save(cat(2,'data_',segmentString,'_raw.mat'), 'timeMatrix',...
                            'contextMatrix','includedFromSegment');
timeMatrix = cat(3, truthTimeMatrix, roadMatrix(:,afterRange));
contextMatrix = directionLabels;
save(cat(2,'truth_',segmentString,'_raw.mat'), 'timeMatrix',...
                            'contextMatrix','includedFromSegment');
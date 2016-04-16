clear;
% Takes segment_XXX.mat files and transforms them into road coordinates
% Uses recursion to find closest road - but currently can't handle lane
% changes well
% 4/11/16
%%
segmentString = 's3sb2';
%%
load(cat(2,'segment_',segmentString,'.mat'));
load('roads_all.mat');

importUsefulFunctions();

nobs = size(timeMatrix,1);

todo = 1:nobs;
currentPoint = 101;
nBefore = 100;
nAfter = 50;
beforeRange = currentPoint-nBefore:currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;

norm2d = @(vec) sum(vec.^2, 2).^.5;

dataTimeMatrix = zeros(length(todo),nBefore+1,3);
truthTimeMatrix = zeros(length(todo),nAfter,3);
includedFromSegment = ones(length(todo),1);
directionLabels = zeros(length(todo),1);
for obsidx = 1:length(todo)
    obs = todo(obsidx);
    
    path = squeeze(timeMatrix(obs, beforeRange, 1:2));
    
    % find the road that matches this trajectory
    [roadsBefore, closestTraj, closestGradient] = findRoad(path,'backwards');
    
    distance = path - closestTraj;
    lateralB = distance(:,1) .* closestGradient(:,2) -...
        distance(:,2) .* closestGradient(:,1);
    longitudinalB = [0; cumsum(norm2d(diff(closestTraj)))];
    dataTimeMatrix(obsidx,:,:) = [longitudinalB, lateralB, roadsBefore];
    
    %after
    path = squeeze(timeMatrix(obs, afterRange, 1:2));
    % find the road that matches this trajectory
    [roadsAfter, closestTraj, closestGradient] = findRoad(path,'forwards');
    
    distance = path - closestTraj;
    lateralA = distance(:,1) .* closestGradient(:,2) -...
        distance(:,2) .* closestGradient(:,1);
    longitudinalA = [0; cumsum(norm2d(diff(closestTraj)))];
    longitudinalA = longitudinalA + longitudinalB(length(beforeRange));
    truthTimeMatrix(obsidx,:,:) = [longitudinalA, lateralA, roadsAfter];
        
    % maneuver labelling
    if includedFromSegment(obsidx)
        directionLabels(obsidx) = labelDirection(roadsBefore, roadsAfter);
    end
end
dataTimeMatrix = dataTimeMatrix(includedFromSegment>0,:,:);
truthTimeMatrix = truthTimeMatrix(includedFromSegment>0,:,:);
directionLabels = directionLabels(includedFromSegment>0);

%% save
timeMatrix = dataTimeMatrix;
contextMatrix = [];
save(cat(2,'data_',segmentString,'_roadCoord.mat'), 'timeMatrix',...
                            'contextMatrix','includedFromSegment');
timeMatrix = truthTimeMatrix;
contextMatrix = directionLabels;
save(cat(2,'truth_',segmentString,'_roadCoord.mat'), 'timeMatrix',...
                            'contextMatrix','includedFromSegment');
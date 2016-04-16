clear;
% Takes segment_XXX.mat files and transforms them into road coordinates
% uses labeled roads
% 4/15/16
%%
segmentString = 's3sb3';
%%
load(cat(2,'segment_',segmentString,'.mat'));

importUsefulFunctions();

nobs = size(timeMatrix,1);

todo = 1:nobs;
currentPoint = 101;
nBefore = 100;
nAfter = 50;
beforeRange = currentPoint-nBefore:currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;

dataTimeMatrix = zeros(length(todo),nBefore+1,2);
truthTimeMatrix = zeros(length(todo),nAfter,2);
includedFromSegment = zeros(length(todo),1);
directionLabels = zeros(length(todo),1);
for obsidx = 1:length(todo)
    obs = todo(obsidx);
    
    %before
    path = squeeze(timeMatrix(obs, beforeRange, 1:2));
    roadsBefore = squeeze(timeMatrix(obs,beforeRange,5));
    [longlatBefore, successBefore] = findRoadCoords(roadsBefore, path);
    dataTimeMatrix(obsidx,:,:) = longlatBefore;
    
    %after
    path = squeeze(timeMatrix(obs, afterRange, 1:2));
    roadsAfter = squeeze(timeMatrix(obs, afterRange, 5));
    [longlatAfter, successAfter] = findRoadCoords(roadsAfter, path);
    longlatAfter(:,1) = longlatAfter(:,1) + longlatBefore(end,1);
    truthTimeMatrix(obsidx,:,:) = longlatAfter;
    
    % maneuver labelling
    if successBefore && successAfter
        includedFromSegment(obsidx) = 1;
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
save(cat(2,'data_',segmentString,'_roadCoord.mat'), 'timeMatrix',...
                            'contextMatrix','includedFromSegment');
timeMatrix = cat(3, truthTimeMatrix, roadMatrix(:,afterRange));
contextMatrix = directionLabels;
save(cat(2,'truth_',segmentString,'_roadCoord.mat'), 'timeMatrix',...
                            'contextMatrix','includedFromSegment');

                        
%% plotting differences
% load('data2_s3sb2_roadTrue.mat')
% truet = timeMatrix;
% load('data_s3sb2_roadCoord.mat')
% truem = squeeze(truet(uncounted==0,:,2));
% mm = squeeze(timeMatrix(uncounted==0,:,2));
% mm = abs(mm);
% truem = abs(truem);
% figure(1); clf; hold on;
% plot((1:size(mm,2))/10, mean(mm,1), 'r-');
% plot((1:size(truem,2))/10, mean(truem,1), 'g-');
% title('difference in road matching errors in seg s3sb2');
% legend('findRoad','NGSIM labels');
% ylabel('m'); xlabel('s');
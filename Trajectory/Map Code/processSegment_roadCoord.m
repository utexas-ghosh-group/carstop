clear;
% Takes segment_XXX.mat files and transforms them into road coordinates
% uses labeled roads
% 4/18/16
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
includedFromSegment = zeros(length(todo),1);
directionLabels = zeros(length(todo),1);
for obsidx = 1:length(todo)
    obs = todo(obsidx);
    
    path = squeeze(timeMatrix(obs,:,1:2));
    roadsHere = squeeze(timeMatrix(obs,:,5));
    [longlat, success] = findRoadCoords(roadsHere, path);
    dataTimeMatrix(obsidx,:,:) = longlat(beforeRange,:);
    truthTimeMatrix(obsidx,:,:) = longlat(afterRange,:);
    
    if success
        includedFromSegment(obsidx) = 1;
        directionLabels(obsidx) = labelDirection(roadsHere(beforeRange),...
                                                 roadsHere(afterRange));
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

% plug into roadCoords2rawCoords plots
% load('data_rand1k_roadCoord.mat','timeMatrix');
% dataMatrix = timeMatrix;
% load('truth_rand1k_roadCoord.mat');
% timeMatrix = cat(2, dataMatrix, timeMatrix);
% paths = roadCoords2rawCoords(timeMatrix);
% load('segment_rand1k.mat','timeMatrix');
% timeMatrix = timeMatrix(includedFromSegment>0, :, [1,2,5]);
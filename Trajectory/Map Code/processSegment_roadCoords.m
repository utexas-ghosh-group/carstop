clear;
% Takes segment_XXX.mat files and transforms them into road coordinates
% 4/7/16
%%
segmentString = 's3sb2';
%%
load(cat(2,'segment_',segmentString,'.mat'));
load('roads_all.mat');

importUsefulFunctions();

%nobs = size(vehicleMatrix,1);
nobs = size(timeMatrix,1);

todo = 1:500;
currentPoint = 101;
nBefore = 100;
nAfter = 50;
beforeRange = currentPoint-nBefore:currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;

norm2d = @(vec) sum(vec.^2, 2).^.5;

transformed = zeros(size(timeMatrix,1),size(timeMatrix,2),2);
%roadMatrix = zeros(length(todo),size(timeMatrix,2));
guessedRoad = zeros(length(todo),size(timeMatrix,2));
for obs = todo
    
    path = squeeze(timeMatrix(obs, beforeRange, 1:2));
    
    % find the road that matches this trajectory
    [roadsBefore, closestTraj, closestGradient] = findRoad(path,'backwards');
    
    distance = path - closestTraj;
    %lateral = norm2d(distance); % absolute value
    lateralB = distance(:,1) .* closestGradient(:,2) -...
        distance(:,2) .* closestGradient(:,1);
    longitudinalB = [0; cumsum(norm2d(diff(closestTraj)))];
    
    %after
    path = squeeze(timeMatrix(obs, afterRange, 1:2));
    % find the road that matches this trajectory
    [roadsAfter, closestTraj, closestGradient] = findRoad(path,'forwards');
    
    distance = path - closestTraj;
    %lateral = norm2d(distance); % absolute value
    lateralA = distance(:,1) .* closestGradient(:,2) -...
        distance(:,2) .* closestGradient(:,1);
    longitudinalA = [0; cumsum(norm2d(diff(closestTraj)))];
    longitudinalA = longitudinalA + longitudinalB(length(beforeRange));

    transformed(obs,:,:) = [longitudinalB, lateralB; ...
                                     longitudinalA, lateralA];
    guessedRoad(obs,:) = [roadsBefore; roadsAfter];
    
    % for labelling

%     labelRange = currentPoint+nAfter + (0:-1:-10);
%     path = squeeze(timeMatrix(obs, labelRange, :));
%     [labelRoad,~,~] = findRoad(path, 'backwards');
%     label(obs,:) = labelRoad;
end
roadMatrix = timeMatrix(todo,:,5);

%% save
timeMatrix = transformed;
save(cat(2,'data_',segmentString,'_roadCoord.mat'),'timeMatrix');
save(cat(2,'roads_',segmentString,'_roadCoord.mat'),'roadMatrix','guesedRoad');
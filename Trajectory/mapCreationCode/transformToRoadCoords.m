clear;
load('segment_s2nb_2.mat');
load('roads_all.mat');

nobs = size(vehicleMatrix,1);

todo = 1:nobs;
currentPoint = 71;
nBefore = 70;
nAfter = 70;
beforeRange = currentPoint-nBefore:currentPoint;
afterRange = currentPoint+1 : currentPoint+nAfter;

norm2d = @(vec) sum(vec.^2, 2).^.5;

transformed = zeros(size(timeMatrix));
roadMatrix = zeros(size(timeMatrix,1),2);
guess = zeros(nobs,70,2);
for obs = todo
    
    path = squeeze(timeMatrix(obs,beforeRange,:));
    
    % find the road that matches this trajectory
    [roadBefore, closestTraj, closestGradient] = findClosestRoad(path,'backwards');
    
    distance = path - closestTraj;
    %lateral = norm2d(distance); % absolute value
    lateralB = distance(:,1) .* closestGradient(:,2) -...
        distance(:,2) .* closestGradient(:,1);
    longitudinalB = [0; cumsum(norm2d(diff(closestTraj)))];
    
    %after
    path = squeeze(timeMatrix(obs, afterRange, :));
    % find the road that matches this trajectory
    [roadAfter, closestTraj, closestGradient] = findRoad(path,'forwards');
    
    distance = path - closestTraj;
    %lateral = norm2d(distance); % absolute value
    lateralA = distance(:,1) .* closestGradient(:,2) -...
        distance(:,2) .* closestGradient(:,1);
    longitudinalA = [0; cumsum(norm2d(diff(closestTraj)))];
    longitudinalA = longitudinalA + longitudinalB(length(beforeRange));

    transformed(obs,:,:) = [longitudinalB, lateralB;...
                                     longitudinalA, lateralA];
    roadMatrix(obs,:) = [roadBefore, roadAfter];
end

%% save
timeMatrix = transformed;
save('segment_s2nb2_road.mat','timeMatrix','vehicleMatrix','roadMatrix');

%% plot
% load truth_s2nb_2_nomapFIX.mat;
% colors = hsv(7);
% figure(1);clf;hold on;
% for j = todo
%     %if nextTruth(j) == 4
%         plot(transformed(j,:,1), transformed(j,:,2),'Color',colors(nextTruth(j),:));
%     %end
% end
% plot(path(:,1),path(:,2),'b-',closestTraj(:,1),closestTraj(:,2),'r-');
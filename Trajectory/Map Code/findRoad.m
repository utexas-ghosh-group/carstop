function [closestRoad, closestTrajectory, closestGradient] = ...
    findRoad(trajectory, direction)
% 4/7/16
%% testing
% clear;
% temp = load('segment_s2nb_2.mat');
% trajectory = squeeze(temp.timeMatrix(3,1:71,:));
% %trajectory = trajectory(37:50,:);
% clear temp;
% direction = 'backwards';
% i=7
%%

roadStruct = load('roads_all.mat');
global roads;
roads = roadStruct.roads;
global transitions;
transitions = roadStruct.transitions;
clear roadStruct;

nPoints = size(trajectory, 1);
nRoads = length(roads);
backwards = strcmp(direction,'backwards');

% initial point
if backwards
    startingPoint = trajectory(nPoints,:);
else
    startingPoint = trajectory(1,:);
end

lowestError = 20;
closestTrajectory = [];
closestGradient = [];
for i = 1:nRoads
    
    road = roads(i).segments;
    distances = 1:size(road,1);
    for j = 1:size(road,1)
        thisDist = startingPoint - closestPoint(road(j,:),startingPoint);
        distances(j) = norm(thisDist);
    end
    [bestFP,chosenSegment] = min(distances);
    
    if bestFP <= lowestError   % otherwise not worth checking out
        [outTraj, outGrad, outError, outRoads, ~] = recurseRoad(trajectory, ...
            road(chosenSegment,:), i, chosenSegment, 0, lowestError, ...
            nPoints, backwards, []);
        if outError < lowestError   % new minimum!
            closestRoad = outRoads;
            closestTrajectory = outTraj;
            closestGradient = outGrad;
            lowestError = outError;
        end
    end
    
end

% plot(trajectory(:,1),trajectory(:,2),'b-',...
%    closestTrajectory(:,1),closestTrajectory(:,2),'r-');
function [closestTraj, closestGradient, err] = ...
        recurseRoad(trajectory, segment, roadNum, segmentNum, err,...
                    minErr, nPoints, backwards, alreadySplitList)
% assumes global 'roads' , 'transitions'

    nLeft = size(trajectory,1);
    if backwards
        point = trajectory(nLeft,:);
        trajectory = trajectory(1:nLeft-1,:);
    else
        point = trajectory(1,:);
        trajectory = trajectory(2:nLeft,:);
    end
    
    % add this point and recalculate error
    [currentPoint, currentGradient] = closestPoint(segment, point);
    currentDistance = norm(point - currentPoint);
    err = calculateError(err, currentDistance, nLeft, nPoints);
    
    
    % check if finished
    if isempty(trajectory)
        closestTraj = currentPoint;
        closestGradient = currentGradient;
        return
    end
    
    % check for early stopping
    if err > minErr
        closestTraj = zeros(size(trajectory));
        closestGradient = closestTraj;
        return
    end

    % continue
    if backwards
        nextPoint = trajectory(size(trajectory,1),:);
        segment = shortenRoad(segment, currentPoint, 'before');
        [nextSegments, nextRoadNums, nextSegNums] = ...
                getPreviousSegments(roadNum, segmentNum, segment);
    else
        nextPoint = trajectory(1,:);
        segment = shortenRoad(segment, currentPoint, 'after');
        [nextSegments, nextRoadNums, nextSegNums] = ...
                getNextSegments(roadNum, segmentNum, segment);
    end
    
    % have to calculate one ahead, to decide if to split
    currentError = norm(nextPoint - closestPoint(segment, nextPoint));
    include = [];
    for i = 1:size(nextSegments,1)
        nextError = norm(nextPoint - closestPoint(nextSegments(i,:), nextPoint));
        if nextError < currentError && ~any(alreadySplitList==i)
            include = cat(1, include, i);
        end
    end
    splitList = cat(1, alreadySplitList, include);
    

    
    % calculate all next segments
    recurseFunction = @(i) recurseRoad(trajectory, nextSegments(i,:), ...
        nextRoadNums(i), nextSegNums(i), err, minErr, nPoints, ...
        backwards, []);
    [trajectories, gradients, errors] = arrayfun(recurseFunction, ...
                                include, 'UniformOutput',0);
    
    % calculate current segment, if still useful
    if length(splitList) < length(nextSegNums) 
        [curTraj, curGrad, curEr] = recurseRoad(trajectory, segment, ...
        roadNum, segmentNum, err, minErr, nPoints, backwards, splitList);
        addCurrent = length(include)+1;
        trajectories{addCurrent} = curTraj;
        gradients{addCurrent} = curGrad;
        errors{addCurrent} = curEr;
    end
                            
    
    % pick best path
    errors = cell2mat(errors);
    [err, bestPath] = min(errors);
    closestTraj = trajectories{bestPath};
    closestGradient = gradients{bestPath};
    if backwards
        closestTraj = [closestTraj; currentPoint];
        closestGradient = [closestGradient; currentGradient];
    else
        closestTraj = [currentPoint; closestTraj];
        closestGradient = [currentGradient; closestGradient];
    end
end




%
function [nextSegments, nextRoadNums, nextSegNums] = ...
                    getPreviousSegments(roadNum, segmentNum, segment)
    global roads; global transitions;
    thisRoad = roads(roadNum).segments;
    if segmentNum > 1    % previous segment from same road
        nextRoadNums = roadNum;
        nextSegNums = segmentNum - 1;
        nextSegments = thisRoad(nextRoadNums,:);
        
    elseif any(transitions(:,roadNum))  % check incoming roads
        nextRoadNums = find(transitions(:,roadNum));
        nextSegNums = [];
        nextSegments = zeros(0,5);
        for i = nextRoadNums
            thisRoad = roads(i).segments;
            thisSegment = size(roads(i).segments,1);
            nextSegNums = cat(1, nextSegNums, thisSegment);
            nextSegments = cat(1, nextSegments, thisRoad(thisSegment,:));
        end
        
    else   % no roads - draw straight line
        nextRoadNums = roadNum;
        [lastPoint, lastgrad] = startPoint(segment);
        nextSegments = constructSegment(lastPoint -...
            lastgrad*100,lastPoint);
        nextSegNums = 1;
    end
end

function [nextSegments, nextRoadNums, nextSegNums] = ...
                    getNextSegments(roadNum, segmentNum, segment)
    global roads; global transitions;
    thisRoad = roads(roadNum).segments;
    if segmentNum < size(thisRoad,1) % next segment from same road
        nextRoadNums = roadNum;
        nextSegNums = segmentNum + 1;
        nextSegments = thisRoad(nextRoadNums,:);
        
    elseif any(transitions(roadNum,:)) % next roads
        nextRoadNums = find(transitions(roadNum,:));
        nextSegNums = [];
        nextSegments = zeros(0,5);
        for i = nextRoadNums
            thisRoad = roads(i).segments;
            thisSegment = size(roads(i).segments,1);
            nextSegNums = cat(1, nextSegNums, thisSegment);
            nextSegments = cat(1, nextSegments, thisRoad(thisSegment,:));
        end
        
    else  % no further segments, just continue linearly
        nextRoadNums = roadNum;
        [lastPoint, lastgrad] = endPoint(segment);
        nextSegNums = 100;
        nextSegments = constructSegment(lastPoint, ...
            lastPoint + lastgrad*100);
    end
end
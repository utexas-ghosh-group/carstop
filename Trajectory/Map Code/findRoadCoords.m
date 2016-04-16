function [path,success] = findRoadCoords(vroads, path)
% given a path and road labels, find closest coordinates on road
% transform into longitudinal/lateral framework
% 4/15/16
global roads;
success = 0;
closestTraj = path;
closestGradient = path;
longitudinalB = zeros(length(path),1);

if any(vroads == 0)
    return
end
prevSegment = vroads(1);
completedRoad = 0;
for j = 1:length(path)
    thisRoad = vroads(j);
    segments = roads(thisRoad).segments;
    err = 5;
    for k = 1:size(segments,1)
        [trajAtPoint, gradAtPoint] = closestPoint(segments(k,:),path(j,:));
        newError = norm(path(j,:)-trajAtPoint);
        if newError < err
            err = newError;
            currentTraj = trajAtPoint;
            currentGrad = gradAtPoint;
        end
    end
    if err >= 5
        return
    end
    distance = distanceAlongRoad(segments, currentTraj);
    if thisRoad ~= prevSegment % changed road
        straightLine = currentTraj - prevTraj;
        straightLineDist = straightLine(1)*currentGrad(1)...
                                + straightLine(2)*currentGrad(2);
        toRoadEnd = distanceAlongRoad(roads(prevSegment).segments,'total');
        if toRoadEnd - prevDist < straightLineDist % road ended, probably
            completedRoad = completedRoad + toRoadEnd;
        else % lane change... probably
            completedRoad = completedRoad + prevDist + ...
                            straightLineDist - distance;
        end
        prevSegment = thisRoad;
    end
    longitudinalB(j) = distance + completedRoad;
    prevTraj = currentTraj;
    prevDist = distance;
    closestTraj(j,:) = currentTraj;
    closestGradient(j,:) = currentGrad;
end

deviance = path - closestTraj;
lateralB = deviance(:,1) .* closestGradient(:,2) -...
    deviance(:,2) .* closestGradient(:,1);
path = [longitudinalB, lateralB];

success = 1;
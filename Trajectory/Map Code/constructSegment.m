function segment = constructSegment(startpoint, endpoint)
% linear_segment = [start_x, start_y, -1, distance, angle]
% arc_segment = [origin_x, origin_y, radius, angle, angle_dist]

isSegment = @(vector) all(size(vector)==[1,5]);
isCoord = @(vector) all(size(vector)==[1,2]);
rotationMatrix = @(angle) [cos(angle), -sin(angle); sin(angle), cos(angle)];

if isSegment(startpoint) && isSegment(endpoint)
    %% curve
    [startpt, startGrad] = endPoint(startpoint);
    [endpt, endGrad] = startPoint(endpoint);
    warpangles = linspace(-pi/8, pi/8, 5);
    warpPenalty = 2.^abs(linspace(-1, 1, 5));
    origin = [0,0];
    minDist = 0;
    mincost = 1000;
    for i = 1:length(warpangles)
        for j = 1:length(warpangles)
            startOrthog = startGrad*rotationMatrix(warpangles(i) + pi/2);
            endOrthog = endGrad*rotationMatrix(warpangles(j) + pi/2);
            dists = [startOrthog', endOrthog'] \ (endpt-startpt)';
            cost = abs(diff(abs(dists)))*warpPenalty(i)*warpPenalty(j);
            if cost < mincost
                mincost = cost;
                origin = startpt + dists(1)*startOrthog;
                minDist = mean(abs(dists));
            end
        end
    end
    if minDist > 1000 % just make it linear instead
        dist = endpt - startpt;
        segment = [startpt, -1, norm(dist), atan2(dist(2),dist(1)) ];
    else
    newGrad = startpt - origin;
    rotation = sign(newGrad * [startGrad(2) -startGrad(1)]');
    startAngle = atan2(newGrad(2), newGrad(1));
    endAngle = atan2(endpt(2) - origin(2), endpt(1) - origin(1));
    endAngle = endAngle + 2*pi*rotation;
    angleChange = rem( endAngle - startAngle , 2*pi);
    segment = [origin, minDist, startAngle, angleChange];
    end
    
elseif isCoord(startpoint) && isCoord(endpoint)
    %% linear
    dist = endpoint - startpoint;
    segment = [startpoint, -1, norm(dist), atan2(dist(2),dist(1)) ];
end

end
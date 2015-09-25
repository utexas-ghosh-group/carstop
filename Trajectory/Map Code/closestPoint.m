function [closestpoint, gradient] = closestPoint(segment, point)

if isempty(segment)  % null segment, returns very distant point
    closestpoint = point + [10000, 10000];
    gradient = [0,0];

elseif segment(3) < 0
    
    startpt = segment(1:2);
    endpt = endPoint(segment);
    gradient = ang2vec(segment(5));
    if (point-endpt) * gradient' > 0
        % angle from end has positive dot prod. with segment
        closestpoint = endpt;
    elseif (point-startpt) * gradient' < 0
        % angle toward start is aligned with segment
        closestpoint = startpt; %start point
    else
        % neither angle aligns, closest point is interior
        closestpoint = segment(1:2) + gradient *...
            ((point - segment(1:2)) * gradient');
    end
else
    
    shiftedpoint = point - segment(1:2);
    angle = atan2(shiftedpoint(2),shiftedpoint(1));
    oppositeAngle = (segment(4)+segment(5))/2 + pi;
    if within(angle, segment(4), segment(5))
        closestpoint = segment(1:2) + segment(3)*[cos(angle) sin(angle)];
        gradient = ang2vec( angle + pi/2*sign(segment(5)) );
    elseif within(angle, segment(4), oppositeAngle)
        [closestpoint, gradient] = startPoint(segment);
    else
        [closestpoint, gradient] = endPoint(segment);
    end
end
end

function yn = within(angle, start, span)
    if start + span > pi
        span = [pi-start, start+span-pi];
        start = [start, -pi];
    elseif start + span < -pi
        span = [-pi-start, start+span+pi];
        start = [start, pi];
    end
    yn = any(abs(angle-start-span/2)<=abs(span/2));
end
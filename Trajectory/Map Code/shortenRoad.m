function road = shortenRoad(road, point, part)
% input 1: road segment or matrix where rows are road segments
% input 2: point at which to cut
% if point is not actually on the segment, will return something crazy...
% input 3: 'before' or 'after'
% output : the road up to or after the point, as a matrix of road segments

if size(road,1) > 1
    mindist = 100;
    closest =0;
    for i = 1:size(road,1)
        dist = norm(point - closestPoint(road(i,:), point));
        if dist < mindist
            mindist = dist;
            closest = i;
        end
    end
else
    closest = 1;
end
segment = road(closest,:);

    if strcmp( part, 'before')
    if AboutEqual(startPoint(segment), point, .001)
        if segment(3) < 0
            segment(4) = 0;
        else
            segment(5) = 0;
        end
        
    elseif ~AboutEqual(endPoint(segment), point, .001)
    
    if segment(3) < 0
        segment(4) = sum((point - segment(1:2)).^2).^.5;
    else  % use law of cosines to cut angle
        newAngle = acos(1 - ...
            sum((point - startPoint(segment)).^2) ...
            /segment(3)^2 / 2);
        segment(5) = newAngle * sign(segment(5));
    end
    
    end
    elseif strcmp( part, 'after')
    if AboutEqual(endPoint(segment), point, .001)
        if segment(3) < 0
            segment(1:2) = point;
            segment(4) = 0;
        else
            segment(5) = 0;
        end
       
    elseif ~AboutEqual(startPoint(segment), point, .001)
    
    if segment(3) < 0
        segment(4) = sum((point - endPoint(segment)).^2).^.5;
        segment(1:2) = point;
    else  % use law of cosines to cut angle
        newAngle = acos(1 - ...
            sum((point - endPoint(segment)).^2) ...
            /segment(3)^2 / 2);
        liff = point - segment(1:2);
        segment(4) = atan2(liff(2), liff(1));
        segment(5) = newAngle * sign(segment(5));
    end
    
    end
    end
    
    road(closest,:) = segment;
    if strcmp(part, 'before')
        road = road(1:closest,:);
    else
        road = road(closest:size(road,1),:);
    end
end
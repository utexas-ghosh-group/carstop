function totalLength = distanceAlongRoad(road, point)
    % if point is not on road, results are not guaranteed to make sense
    % if point = "total", computes total length of road
    % 4/15/16

    if strcmp(point,'total')
        totalLength = 0;
        for i = 1:size(road,1)
            if road(i,3) < 0
                totalLength = totalLength + road(i,4);
            else
                totalLength = totalLength + abs(road(i,5)*road(i,3));
            end
        end
        return 
    end
    
    totalLength = 0;
    for i = 1:size(road,1)
        segment = road(i,:);  
        if AboutEqual(point, closestPoint(segment,point))
            if segment(3) < 0
                totalLength = totalLength + norm(point - segment(1:2));
            else
                totalLength = totalLength + acos(1 - ...
                    sum((point - startPoint(segment)).^2) ...
                    /segment(3)^2 / 2) * segment(3); 
            end
            return
        else
            if segment(3) < 0
                totalLength = totalLength + segment(4);
            else
                totalLength = totalLength + abs(segment(5)*segment(3));
            end
        end
    end
end
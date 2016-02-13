function yn = inRectangle(coords, pointA, pointB, width)
    % assumes: coords nX2, points 1X2, width 1X1
    
    % center around point A, to simplify
    coords = coords - repmat(pointA,size(coords,1),1);
    pointB = pointB - pointA;
    magB = norm(pointB,2);
    
    crossProducer = [0 -1; 1 0];
    crossProduct = coords * crossProducer * pointB'; % y1 x2 - x1 y2
    inOrthogonal = (crossProduct / magB <= width / 2) .*...
        (crossProduct / magB >= -width / 2);
    
    dotProduct = coords*pointB';   % x1 y1 + x2 y2
    eps = 1;
    inParallel = (dotProduct <= magB.^2 + eps) .* (dotProduct >= 0 - eps);

    yn = (inOrthogonal .* inParallel) > 0;
end
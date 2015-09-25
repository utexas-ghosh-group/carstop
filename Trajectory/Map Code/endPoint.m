function [point, gradient] = endPoint(segment)
% linear_segment = [start_x, start_y, -1, distance, angle]
% quadratic_segment = [origin_x, origin_y, radius, angle, angle_dist]

if segment(3) < 0 % straight line
    gradient = ang2vec(segment(5));
    point = segment(1:2) + gradient*segment(4);

else % arc
    point = segment(1:2) + segment(3)*[cos(segment(4)+segment(5)),...
            sin(segment(4)+segment(5))];
    orthangle = segment(4) + segment(5) + pi/2*sign(segment(5));
    gradient = ang2vec(orthangle);

end
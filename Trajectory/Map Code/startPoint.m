function [point, gradient] = startPoint(segment)
% linear_segment = [start_x, start_y, -1, distance, angle]
% quadratic_segment = [origin_x, origin_y, radius, angle, angle_dist]

if segment(3) < 0 % straight line
    point = segment(1:2);
    gradient = ang2vec(segment(5));

else % arc
    point = segment(1:2) + segment(3)*[cos(segment(4)),...
            sin(segment(4))];
    orthangle = segment(4) + pi/2*sign(segment(5));
    gradient = ang2vec(orthangle);
end
% 2/2/16
function traj = CV_prediction(path)
% Simple physics prediction, as a baseline model / tester
% import parameters: filterSize, accel_cutoff, nToPredict,
% prevent_negative_speed (not complete yet)
    nToPredict = 70;
    filterSize = 12; % 1 second
    accel_cutoff = 200;
    
    % estimate current parameters
    nT = size(path,1);
    filteredPath = path(nT-filterSize+1:nT, :);
    filteredtimes = ( -filterSize+1:0 )';
    polysX = polyfit(filteredtimes, filteredPath(:,1),2);
    polysY = polyfit(filteredtimes, filteredPath(:,2),2);
    x = [polysX(3), polysY(3)];
    v = [polysX(2), polysY(2)];
    a = [polysX(1), polysY(1)];
    
    % predict future
    dt = (1:nToPredict)';
    t1 = min(dt, accel_cutoff);
    t2 = dt-t1;
    
    expoff = exp(-accel_cutoff);
    
    v1 = repmat(v, length(t1),1) + t1*a - expoff*(exp(t1) - 1)*a;
    
%     if prevent_negative_speed
%         if v < 0
%             %print "CA_physics called with v < 0"
%             v1 = min(v1, 0);
%         end
%         negativePoints = v1 < 0;
%         v1(negativePoints,:) = 0;
%         if a >= 0
%             t0 = 0; %???
%         elseif cutoff > 5
%             t0 = -v/a; % find time where v hits 0
%         else % second-order polynomial approx.
%             t0 = 1/expoff - 1 + np.power(...
%                             np.power(1/expoff-1,2) + 2*v/a/expoff, .5);
%         end
%         t1(negativePoints) = t0;
%     end
    
    x1 = repmat(x, length(t1),1) + t1*v + t1.^2*a/2 -...
                    expoff*(exp(t1) - t1 - 1)*a;
        
    % continue after the cutoff has been passed
    traj = x1 + repmat(t2,1,2).*v1;
end
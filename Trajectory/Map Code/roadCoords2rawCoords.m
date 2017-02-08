function paths = roadCoords2rawCoords(timeMatrix)
% 4/15/16
% basically undoing the process performed in processSegment_roadTrue
importUsefulFunctions();
global roads;
paths = timeMatrix(:,:,1:2);
for i = 1:size(timeMatrix,1)
    roadnums = timeMatrix(i,:,3);
    roadBins = [roadnums(1), 1];
    previousRoad = roadnums(1);
    for j = 2:length(roadnums)
        if roadnums(j) ~= previousRoad
            roadBins = cat(1,roadBins, [roadnums(j), 1]);
            previousRoad = roadnums(j);
        else
            roadBins(size(roadBins,1),2) = roadBins(size(roadBins,1),2)+1;
        end
    end
    position = 1;
    completedRoad = 0;
    lastRoadLength = 0;
    lastDistance = 0;
    for j = 1:size(roadBins,1)
        nvals = roadBins(j,2);
        positions = position : position+nvals-1;
        distances = squeeze(timeMatrix(i,positions,1));
        
        roadSegs = roads(roadBins(j,1)).segments;
        totalRoadLength = distanceAlongRoad(roadSegs,'total');

        %distances = distances - completedRoad;
        straightLineDist = distances(1) - lastDistance;
        if straightLineDist + lastDistance < lastRoadLength + completedRoad
            % probably a lane change
            testspaces = interpolate(roadSegs,.1);
            testdistances = (testspaces(:,1)-lastPoint(1))*lastGrad(1)+...
                             (testspaces(:,2)-lastPoint(2))*lastGrad(2);
            [~,winner] = min((testdistances - straightLineDist).^2);
            completedRoad = lastDistance + straightLineDist -.1*(winner-1);
        else % probably end of the last road
            completedRoad = completedRoad + lastRoadLength;
        end
        distances = distances - completedRoad;
        
        if length(distances)==1 % interpolate() handles single inputs differently
            if distances(nvals) >= totalRoadLength
                [coords,grads] = endPoint(roadSegs(end,:));
            else
                [coords,grads] = interpolate(roadSegs, [0, distances]);
                coords = coords(2,:); grads = grads(2,:);
            end
        else
            [coords,grads] = interpolate(roadSegs, distances);
            if distances(nvals) >= totalRoadLength && size(coords,1)<nvals
                % interpolate() sometimes misses end point...
                [coord,grad] = endPoint(roadSegs(end,:));
                endpointsToAdd = nvals - size(coords,1);
                coords = cat(1, coords, repmat(coord, endpointsToAdd,1));
                grads = cat(1, grads, repmat(grad, endpointsToAdd,1));
            end
        end
        
        paths(i,positions,1) = coords(:,1) + grads(:,2).*timeMatrix(i,positions,2)';
        paths(i,positions,2) = coords(:,2) - grads(:,1).*timeMatrix(i,positions,2)';
        
        % prep for next lane
        position = position + nvals;
        lastPoint = coords(nvals,:);
        lastGrad = grads(nvals,:);
        lastRoadLength = totalRoadLength;
        lastDistance = timeMatrix(i,position-1,1);
    end
end

%% testing, keep commented otherwise
% load('data_rand1k_raw.mat')
% oldIncluded = includedFromSegment;
% load('data_rand1k_roadCoord.mat','includedFromSegment');
% newIncluded = includedFromSegment(oldIncluded > 0);
% timeMatrix = timeMatrix(newIncluded>0,:,:);
% errors = ((paths(:,:,1)-timeMatrix(:,:,1)).^2 +...
%             (paths(:,:,1)-timeMatrix(:,:,1)).^2 ).^.5;
% timesteps = (1:size(timeMatrix,2))/10;
%  %
% figure(1); clf;
% i=11;
% plot(paths(i,:,1),paths(i,:,2),'r-.',...
%      timeMatrix(i,:,1),timeMatrix(i,:,2),'g-.')
%  %
% figure(2); clf; hold on;
% for j = 1:size(timeMatrix,1)
%     plot(1:size(timeMatrix,2), errors(j));
% end
%  %
% figure(3); clf;
% meanerror = mean(errors,1);
% plot(1:size(timeMatrix,2), meanerror);
%  %
% figure(4); clf; hold on;
% percentile = @(vals, fraction) [zeros(1,floor(size(vals,1)*fraction)-1), ...
%     1, zeros(1,ceil(size(vals,1)*(1-fraction)))] * ...
%     sort(vals,1,'descend');
% centiles = [.5 .25 .1];
% color = parula(length(centiles)+1);
% for i = 1:length(centiles)
%     plot(timesteps, percentile(errors, centiles(i)),...
%         'Color',color(i,:));
% end
%  %
% figure(5); clf; hold on;
% meanerror = mean(errors,2);
% hist(meanerror);
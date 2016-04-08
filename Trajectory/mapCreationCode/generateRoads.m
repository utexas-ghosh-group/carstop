clear;
addpath('/home/motrom/Documents/carstop/Trajectory/Map Code')
importUsefulFunctions();
% generate Roads
coords = load('converted.csv');
load('roadnames.mat');

% shift coordinates
%load('intersectionCenters.mat');
%center = intersectionCenters(2,:);
coords = coords(:,1:2);
coords(:,1) = coords(:,1) - 1966000;
coords(:,2) = coords(:,2) - 570000;

%% create road segments
roads = struct('name',[],'segments',[0 0 0 0 0]);
counter = 1;
while size(names,1) > 0
    first = names{1};
    roadName = first(1:length(first)-2);
    roads(counter).name = roadName;
    
    included = 1;
    positions = str2num(first(length(first)));
    % gather all points related to this road
    for i = 2:length(names)
        line = names{i};
        if strcmp(roadName, line(1:length(line)-2)) 
            pos = str2num(line(length(line)));
            included = cat(1, included, i);
            positions = cat(1, positions, pos);
        end
    end
    
    % gather coordinates for this road, in order
    [~,positions] = sort(positions,'ascend');
    theseCoords = coords(included(positions),:);
    ncoords = size(theseCoords,1);
    % make linear/arc segments out of coordinates
    if ncoords < 2
        disp('- - single coordinate for a road - -');
    elseif ncoords == 2
        roads(counter).segments = constructSegment(...
            theseCoords(1,:),theseCoords(2,:));
    elseif ncoords == 3
        roads(counter).segments = [constructSegment(...
                          theseCoords(1,:),theseCoords(2,:)) ;
                          constructSegment(...
                          theseCoords(ncoords-1,:),theseCoords(ncoords,:))];
    elseif ncoords == 4
        firstsegment = constructSegment(theseCoords(1,:),theseCoords(2,:));
        endsegment = constructSegment(...
            theseCoords(ncoords-1,:),theseCoords(ncoords,:));
        roads(counter).segments = [firstsegment; constructSegment(...
            firstsegment,endsegment); endsegment];
    else
        disp('- - too many coordinates, code not complete - -')
    end
        
    coords(included,:) = [];
    names(included) = [];
    counter = counter + 1;
end
save('roads_basic.mat','roads');

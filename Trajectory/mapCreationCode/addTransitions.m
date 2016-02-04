clear;
load('roads_basic.mat'); % roads = struct with names field
nroads = length(roads);
RepoPath = pwd;
RepoPath = RepoPath(1:length(RepoPath)-length('mapCreationCode'));
addpath([RepoPath 'Map Code'])

%% fill in transitions between current roads from file
transitions = zeros(nroads);

fid = fopen('road_transitions.csv','r');
line = fgetl(fid);
readroads = [];
while ~all(line == -1)
    deze = strsplit(line,',');
    from = [];
    to = [];
    for i = 1:nroads
        if strcmp(deze(1),roads(i).name)
            from = i;
        end
        for j = 2:length(deze)
            if strcmp(deze(j),roads(i).name)
                to = cat(1, to, i);
            end
        end
    end
    transitions(from, to) = 1;
    
    line = fgetl(fid);
end
fclose(fid);

save('roadTransitions_basic.mat','transitions');

%% make each transition its own road, update transition matrix
%load('roads_basic.mat');
%load('roadTransitions_basic.mat');
[from,to] = find(transitions);
transitions(:,:) = 0; % reset
newRoadCount = nroads+1;
for i = 1:length(from)
    beforeSegment = roads(from(i)).segments;
    beforeSegment = beforeSegment(size(beforeSegment,1),:);
    afterSegment = roads(to(i)).segments(1,:);
    
    if AboutEqual(endPoint(beforeSegment) , startPoint(afterSegment), 3)
        % if a transition curve is <3 meters, just remove it
        transitions(from(i), to(i)) = 1;
    else
        roads(newRoadCount).name = ...
            cat(2,roads(from(i)).name, '_X_',roads(to(i)).name);
        roads(newRoadCount).segments = ...
            constructSegment(beforeSegment,afterSegment);
        transitions(from(i), newRoadCount) = 1;
        transitions(newRoadCount,to(i)) = 1;
        newRoadCount = newRoadCount + 1;
    end
end

save('roads_all.mat','roads','transitions');

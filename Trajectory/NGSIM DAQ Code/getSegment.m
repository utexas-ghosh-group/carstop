clear;
importUsefulFunctions()

% dataset parameters
idFeature = 1;
timeFeature = 4; % in milliseconds from some reference time
oldTrajFeatures = 5:6;
originFeature = 10;
destFeature = 11;
locationFeatures = [9,13,14,2]; % lane, section, direction, frame
%trajFeatures = 7:8; % x and y (in meters, hopefully)
trajFeatures = [7,8,16,17];

% timing parameters
timeToCoverBefore = 100;  % 100ms frames
timeToCoverAfter = 50;  % 100ms frames, including the current one
minimumTime = 20;   % trajects with < 4 seconds on either side are not used
timesteps = timeToCoverBefore + timeToCoverAfter + 1;

% segmentation parameters
direction = 4;      % 1=E 2=N 3=W 4=S
% pointA = [-11,270];
% pointB = [2,268.5];
% gradAB = pointB - pointA;
% pointA = pointA + [gradAB(2),-gradAB(1)];
% pointB = pointB + [gradAB(2),-gradAB(1)];
pointA = [626,1044];
pointB = [640,1034];
width = 3;
inSegment = @(coords) inRectangle(coords, pointA, pointB, width);
segmentName = [3, direction, pointA, pointB];

% roadList
roadList = importdata('roadList.txt');
location2num = @(x,xp,xf) road2num(x(2),x(3),x(1),xp(2),xp(3),xp(1),...
                            xf(2),xf(3),xf(1),roadList);

% storage
timeMatrix = zeros(0,timesteps,length(trajFeatures)+1); %vehicle X time X pos
nTooShortTrajects = 0;
nTimeSkipTrajects = 0;
nInterpolations = 0;

%%
temp = importdata('preppedData2.csv');

emergEscape = 0;
while (size(temp,1) > 0) && (emergEscape < 10000)
    
    % search for data in correct zone
    coordInSect = find(inSegment(temp(:,7:8)).*(temp(:,14)==direction), 1);
    if isempty(coordInSect)
        emergEscape = 100000;
        break;
    end
    thisRow = temp(coordInSect, :);
    
    % find the rows that apply to this vehicle
    rowsOfThisVehicle = (temp(:,idFeature) == thisRow(idFeature));
    
    % find all locations for this vehicle -> so that start and end of
    % trajectory can be reliably known
    allLocations = temp(rowsOfThisVehicle, locationFeatures);
    originLocation = allLocations(1,:); % add origin
    originLocation(2) = thisRow(originFeature);
    allLocations = cat(1, originLocation, allLocations);
    destLocation = allLocations(end,:); % add destination
    destLocation(2) = thisRow(destFeature);
    allLocations = cat(1, allLocations, destLocation);
    % gather unique locations by finding timepoints where location changes
    sectionChanges = diff(allLocations(:,2))~=0;
    laneChanges = (diff(allLocations(:,1))~=0) .* ...
                       (allLocations(2:end, 2)>0); % only lane changes in section
    uniqueLocations = cat(1, 1, find(sectionChanges + laneChanges > 0)+1);
    uniqLocations = allLocations(uniqueLocations,:);
    % use road2num to assign locations
    emptyLoc = originLocation*0;
    locations = zeros(size(uniqLocations,1),1);
    locations(1) = location2num(uniqLocations(1,:), emptyLoc, emptyLoc);
    locations(end) = location2num(uniqLocations(end,:), emptyLoc, emptyLoc);
    for thisloc = 2:size(uniqLocations,1)-1
        locations(thisloc) = location2num(uniqLocations(thisloc,:),...
                                          uniqLocations(thisloc-1,:),...
                                          uniqLocations(thisloc+1,:));
    end
    uniqueLocations = uniqueLocations -1;
    if uniqueLocations(1) == 0
        uniqueLocations = uniqueLocations(2:end);
    end
    if uniqueLocations(end) > sum(rowsOfThisVehicle)
        uniqueLocations = uniqueLocations(1:end-1);
    end
    locationFrames = allLocations(uniqueLocations,4);
    
    % find rows in the correct time frames
    reject = 0;
    
    rowsBefore = ((temp(:,2) < thisRow(2)) .*...
                  (temp(:,2) >= thisRow(2) - timeToCoverBefore) .*...
                  rowsOfThisVehicle ) > 0;
    vDataBefore = temp(rowsBefore, :);
    currentLen = size(vDataBefore,1);
    beginningInterpolated = timeToCoverBefore - currentLen;
    if currentLen < minimumTime % too short to even interpolate
        reject = 1;
        nTooShortTrajects = nTooShortTrajects + 1;
    elseif currentLen < timeToCoverBefore
        vDataBefore = Interpolate(vDataBefore, beginningInterpolated, 1);
        nInterpolations = nInterpolations + 1;
    end
    
    rowsAfter = ((temp(:,2) > thisRow(2)) .*...
                 (temp(:,2) <= thisRow(2) + timeToCoverAfter) .*...
                 rowsOfThisVehicle ) > 0;
    vDataAfter = temp(rowsAfter, :);
    currentLen = size(vDataAfter,1);
    endInterpolated = timeToCoverAfter - currentLen;
    if currentLen < minimumTime
        reject = 1;
        nTooShortTrajects = nTooShortTrajects + 1;
    elseif currentLen < timeToCoverAfter
        vDataAfter = Interpolate(vDataAfter, endInterpolated, 0);
        nInterpolations = nInterpolations + 1;
    end
    
    vData = cat(1,vDataBefore, thisRow, vDataAfter);
    
    % remove all data from temp
    rowsIncluded = rowsAfter + rowsBefore;
    rowsIncluded(coordInSect) = 1;
    temp = temp(~rowsIncluded, :);
    % remove trajectories that are too short
%     reject = 0;
%     if size(vData,1) < timesteps  % don't have coverage
%         nTooShortTrajects = nTooShortTrajects + 1;
%         reject = 1;
%     end
    % checks that probably aren't needed
%     if any(diff(vData(:,4)) ~= 100) % check for skipped timesteps
%        nTimeSkipTrajects = nTimeSkipTrajects + 1;
%        reject = 1;
%     end  
%     [~,sorter] = sort(vData(:,2)); % check for unsorted data
%     if ~all(sorter' == 1:length(sorter))
%        nTimeSkipTrajects = nTimeSkipTrajects + 1;
%     end
%     vData = vData(sorter,:);
    
    if ~reject
        timeRow = zeros(1,timesteps, length(trajFeatures)+1);
        timeRow(1,:,1:length(trajFeatures)) = vData(:,trajFeatures);
        for time = 1:timesteps
            timeRow(1, time, length(trajFeatures)+1) = ...
                    locations(sum(locationFrames <= vData(time,2))+1);
        end

        timeMatrix = cat(1,timeMatrix, timeRow);
        emergEscape = emergEscape + 1;
    end
end

save('segment_s3sb3.mat','timeMatrix','segmentName');
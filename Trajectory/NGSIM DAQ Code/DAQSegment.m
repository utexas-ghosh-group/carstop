clear;

% dataset parameters
idFeature = 1;
timeFeature = 4; % in milliseconds from some reference time
oldtrajFeatures = 5:6;  % x and y (in meters, hopefully)
trajFeatures = 7:8;  % x and y (in meters, hopefully)
transientFeatures = [9,12:15]; % the descriptor features that can change
                               % over time, such as lane number
                               % storing at beginning and at end of traj.
setFeatures = [10,11]; % the descriptor features that won't change

% timing parameters
timeToCoverBefore = 70;  % 100ms frames
timeToCoverAfter = 70;  % 100ms frames, including the current one
minimumTime = 20;   % trajects with < 2 seconds on either side are not used
timesteps = timeToCoverBefore + timeToCoverAfter + 1;

% segmentation parameters
direction = 4;      % 1=E 2=N 3=W 4=S
pointA = [-11,270];
pointB = [2,268.5];
width = 3;
inSegment = @(coords) inRectangle(coords, pointA, pointB, width);
segmentName = [3, direction, pointA, pointB];

% center x,y data for each intersection
intersection = 2;

% storage
timeMatrix = zeros(0,timesteps,length(trajFeatures)); %vehicle X time X pos
vehicleMatrix = zeros(0,length(transientFeatures)*3+length(setFeatures) + 2);
nTooShortTrajects = 0;
nTimeSkipTrajects = 0;
nInterpolations = 0;

%%
temp = importdata('preppedData.csv');

emergEscape = 0;
while (size(temp,1) > 0) && (emergEscape < 10000)
    
    % search for data in correct zone
    coordInSect = find(inSegment(temp(:,5:6)).*(temp(:,14)==direction), 1);
    if isempty(coordInSect)
        emergEscape = 100000;
        break;
    end
    thisFrame = temp(coordInSect, 2);
    
    % find the rows that apply to this vehicle
    rowsOfThisVehicle = (temp(:,idFeature) == temp(coordInSect,idFeature));
  
    % find rows in the correct time frames
    reject = 0;
    
    rowsBefore = ((temp(:,2) < thisFrame) .*...
                  (temp(:,2) >= thisFrame - timeToCoverBefore) .*...
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
    
    rowsAfter = ((temp(:,2) > thisFrame) .*...
                 (temp(:,2) <= thisFrame + timeToCoverAfter) .*...
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
    
    vData = cat(1,vDataBefore, temp(coordInSect,:), vDataAfter);
    
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
    
    vehicleRow = zeros(1,size(vehicleMatrix,2));
    featurecount = 1;
    for vfeat = setFeatures
        if any(vData(:,vfeat) ~= vData(1,vfeat))
            disp('origin/dest change');
        end
        vehicleRow(featurecount) = vData(1, vfeat);
        featurecount = featurecount + 1;
    end
    for vfeat = transientFeatures
        vehicleRow(featurecount) = vData(2, vfeat);
        featurecount = featurecount + 1;
        vehicleRow(featurecount) = vData(timeToCoverBefore+1, vfeat);
        featurecount = featurecount + 1;
        vehicleRow(featurecount) = vData(size(vData,1), vfeat);
        featurecount = featurecount + 1;
    end
    vehicleRow(featurecount) = beginningInterpolated;
    vehicleRow(featurecount+1) = endInterpolated;
    
    timeRow = zeros(1,timesteps, length(trajFeatures));
    timeRow(1,:,:) = vData(:,trajFeatures);
    
    timeMatrix = cat(1,timeMatrix, timeRow);
    vehicleMatrix = cat(1,vehicleMatrix,vehicleRow);
    emergEscape = emergEscape + 1;
    end
end

% center around each intersection
load('intersectionCenters.mat');
timeMatrix(:,:,1) = timeMatrix(:,:,1) - intersectionCenters(intersection,1);
timeMatrix(:,:,2) = timeMatrix(:,:,2) - intersectionCenters(intersection,2);

save('segment_s3sb_2.mat','timeMatrix','vehicleMatrix','segmentName');
clear;

idFeature = 1;
timeFeature = 4; % in milliseconds from some reference time
%trajFeatures = 5:6;  % x and y (in meters, hopefully)
trajFeatures = 7:8;  % x and y (in meters, hopefully)
vehicleFeatures = 9:15; % descriptors
timeToCover = 7 ;  % seconds
timesteps = floor(timeToCover * 1000 / 100);

timeMatrix = zeros(0,timesteps,length(trajFeatures)); %vehicle X time X pos
vehicleMatrix = zeros(0,length(vehicleFeatures));  %vehicle X info
nTooShortTrajects = 0;

%%

temp = importdata('preppedData.csv');

emergEscape = 0;
while (size(temp,1) > 0) && (emergEscape < 100000)
    % remove one vehicle's data
    rowsOfAnID = (temp(:,idFeature) == temp(1,idFeature));
    vData = temp(rowsOfAnID,:);
    temp = temp(~rowsOfAnID,:);
    
    % search for section 3, southbound
    selectedPortion = (vData(:,13) == 3).*(vData(:,14) == 4) > 0;
    
    % gather some time of data after first entry in s3-SB
    minFrame = max(vData(:,2))+1;
    if sum(selectedPortion) > 0
       selectedData = vData(selectedPortion > 0,:);
       minFrame = min(selectedData(:,2));
    end
    timeBackwards = 0;
    selection = (vData(:,2) >= minFrame - timeBackwards).*...
        (vData(:,2) < minFrame+timesteps - timeBackwards) > 0;
    if sum(selection) > 0
        vData = vData(selection,:);
        [~,sorter] = sort(vData(:,2));
        vData = vData(sorter,:);
    else
        vData = [];
    end
        
    % cut data to first 5 seconds
    reject = 0;
    if size(vData,1) < timesteps  % don't have coverage
        if size(vData,1) > 0
            nTooShortTrajects = nTooShortTrajects + 1;
        end
        reject = 1;
    end
    if ~reject
    
    vehicleRow = zeros(1,size(temp,2));
    for vfeat = vehicleFeatures
        % One vehicle I looked at had a weird problem where
        % the very first feature was different than the rest.
        % In addition to this, things like Lane number can change...
        % so only using first second to determine these params.
        if false
        %if sum(vData(1:10,vfeat) ~= vData(1,vfeat)) > 0
            [vals,counts] = DiscreteHist(vData(1:10,vfeat));
            [~,bestval] = max(counts);
            vehicleRow(vfeat) = vals(bestval); %pick most common value
        else
            vehicleRow(vfeat) = vData(1, vfeat);
        end
    end
    vehicleRow = vehicleRow(vehicleFeatures);
    
    timeRow = zeros(1,timesteps, length(trajFeatures));
    timeRow(1,:,:) = vData(:,trajFeatures);
    
    timeMatrix = cat(1,timeMatrix, timeRow);
    vehicleMatrix = cat(1,vehicleMatrix,vehicleRow);
    emergEscape = emergEscape + 1;
    end
end
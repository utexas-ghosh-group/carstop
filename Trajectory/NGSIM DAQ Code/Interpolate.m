function vData = Interpolate(vData, timeToFill, before)

timeFeatures = [2,4:8];
interpTime = 5; % half a second

currentLen = size(vData,1);
if interpTime > currentLen
    disp('vData to short for interpolate to work!');
end

% get velocity (averaged) from a vector
%velocity = @(data,n) conv(data, [1 zeros(1,n-1) -1] / n, 'valid');

if before
    newdata = repmat(vData(1,:), timeToFill, 1);
    for feature = timeFeatures
        vel = (vData(1,feature) - vData(interpTime,feature))...
            / (interpTime-1);
        newdata(:,feature) = vData(1,feature) + (timeToFill:-1:1)*vel;
    end
    vData = cat(1,newdata,vData);
else    
    newdata = repmat(vData(currentLen,:), timeToFill, 1);
    for feature = timeFeatures
        vel = (vData(currentLen,feature) - ...
            vData(currentLen - interpTime + 1,feature)) / (interpTime-1);
        newdata(:,feature) = vData(currentLen,feature) + (1:timeToFill)*vel;
    end
    vData = cat(1,vData,newdata);
end
end
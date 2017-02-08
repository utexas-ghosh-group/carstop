function trajectories = testkmean(mps, timeMatrix,~,timesToPredict)

nMps = size(mps,1);
nobs = size(timeMatrix,1);
trainSize = size(mps,2) - length(timesToPredict); % # obs for matching cluster
prevMps = mps(:,1:trainSize,:);
afterMps = mps(:,trainSize+1:end,:);

trajectories = zeros(nobs,length(timesToPredict),2);
errs = zeros(nMps,1);
for obs = 1:nobs
    prevtraj = timeMatrix(obs,end-trainSize+1:end,:);
    for k = 1:nMps
        err = prevtraj-prevMps(k,:,:);
        errs(k) = sum(sum(err.^2));
    end
    [~,bestclust] = min(errs);
    
    trajectories(obs,:,:) = afterMps(bestclust,:,:);
end
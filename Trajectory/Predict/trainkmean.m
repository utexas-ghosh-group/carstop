function predictor = trainkmean(timeBefore, ~ , timeAfter)
%4/26/16
if size(timeBefore,2) > 30
    timeBefore = timeBefore(:,end-29:end,:);
end

timeMatrix = cat(2, timeBefore, timeAfter);
timeMatrix = timeMatrix(:,:,1:2);
nobs = size(timeMatrix,1);
ntime = size(timeMatrix,2);

nclust=7;
numiter = 50;
clusters = zeros(nobs,1);
clustermeans = zeros(nclust,ntime,2);

% initialization
meanStart = squeeze(mean(timeMatrix(:,1,:),1));
meanEnd = squeeze(mean(timeMatrix(:,ntime,:),1));
vector = meanEnd - meanStart;
vmag = (vector(1)^2+vector(2)^2)^.5;
vangle = atan2(vector(2),vector(1));
for k = 1:nclust
    angle = vangle + pi/12*(k-floor(nclust/2+1));
    newEnd = meanStart + vmag*[cos(angle); sin(angle)];
    clustermeans(k,:,1) = linspace(meanStart(1),newEnd(1),ntime);
    clustermeans(k,:,2) = linspace(meanStart(2),newEnd(2),ntime);
end

for iter = 1:numiter
    % M
    errs = zeros(nclust,1);
    for obs = 1:nobs
        for k = 1:nclust
            err = squeeze(clustermeans(k,:,:)-timeMatrix(obs,:,:));
            errs(k) = sum(sum(err.^2,2),1);
        end
        [~,bestcluster] = min(errs);
        clusters(obs) = bestcluster;
    end
    
    % E
    for k = 1:nclust
        clustermeans(k,:,:) = mean(timeMatrix(clusters==k,:,:),1);
    end
end

includeclust = ones(nclust,1);
for k = 1:nclust
    if ~any(clusters==k)
       includeclust(k) = 0; 
    end
end
predictor = clustermeans(includeclust>0,:,:);

figure(1);clf;hold on;
for k=1:nclust
    plot(clustermeans(k,:,1),clustermeans(k,:,2));
end
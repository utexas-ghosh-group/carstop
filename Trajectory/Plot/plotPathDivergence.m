% This file just has a variety of useful plots

distVector = ((pathA(:,1) - pathB(:,1)).^2 +...
              (pathA(:,2) - pathB(:,2)).^2 ).^.5;
     
figure(1); clf;
plot(pathA(:,1), pathA(:,2),'b.',pathA(1,1),pathA(1,2),'bo',...
     pathB(:,1), pathB(:,2),'r.',pathB(1,1),pathB(1,2),'ro');
legend('path A','starting point','path B','starting point');
title('Two trajectories');


figure(2); clf;
plot((1:length(distVector))*.1,distVector);
title('Distance between points on paths');
xlabel('time (s)');
ylabel('Euclidean distance (m)');



figure(3); clf; hold on;
for obs = zz(1:10)'
    pathA = squeeze(truths(obs,1:30,:));
    pathB = squeeze(guesses(obs,1:30,:));
plot(pathA(:,1), pathA(:,2),'b.',pathA(1,1),pathA(1,2),'bo',...
     pathB(:,1), pathB(:,2),'r.',pathB(1,1),pathB(1,2),'ro');
end

figure(4); clf; hold on;
for obs = todo
    pathA = squeeze(truths(obs,:,:));
    pathB = squeeze(guesses(obs,:,:));
    distVector = ((pathA(:,1) - pathB(:,1)).^2 +...
                  (pathA(:,2) - pathB(:,2)).^2 ).^.5;
    plot((1:length(distVector))*.1,distVector);
end
legend('1','2','3','4','5');
title('Distance between points on paths');
xlabel('time (s)');
ylabel('Euclidean distance (m)');



%%%
% truth, prediction1, prediction2, ...
nPredictions = 2;
error1 = ((truth(:,:,1) - prediction1(:,:,1)).^2 +...
          (truth(:,:,2) - prediction1(:,:,2)).^2 ).^.5;
error2 = ((truth(:,:,1) - prediction2(:,:,1)).^2 +...
          (truth(:,:,2) - prediction2(:,:,2)).^2 ).^.5;

ntrajects = size(truth,1);
ntimesteps = size(truth,2);
timesteps = (1:ntimesteps) * 0.1;

percentile = @(vals, fraction) [zeros(1,floor(size(vals,1)*fraction)-1), ...
    1, zeros(1,ceil(size(vals,1)*(1-fraction)))] * ...
    sort(vals,1,'descend');
colors = hsv(nPredictions);

% compare means and percentiles for multiple models
figure(1); clf; hold on;
plot(timesteps, mean(error1),'-','Color',colors(1,:));
plot(timesteps, mean(error2),'-','Color',colors(2,:));
legend('Prediction 1','Prediction 2');
plot(timesteps, percentile(error1, .1),'--','Color',colors(1,:));
plot(timesteps, percentile(error2, .1),'--','Color',colors(2,:));
title('Average and 10th Percentile Error in Prediction');
xlabel('time (s)');
ylabel('trajectory error (m)');

% histogram of errors for single model
edges = [0:2:20,inf];
hist2d = zeros(ntimesteps, length(edges));
for tstep = 1:ntimesteps
    hist2d(tstep,:) = histc(error1(:,tstep),edges);
end
hist2d = hist2d.^.5; % for evening color scale
figure(2); clf; hold on;
colormap('default');
imagesc(timesteps, edges(1:length(edges)-1)+1.5,...
        hist2d(:,1:length(edges)-1)');
colorbar();
xlabel('time (s)');
ylabel('trajectory error (m)');
title('spread of errors from Map-Following');


% splayed percentiles
centiles = [.5 .25 .1 .05];
color = parula(length(centiles)+1);
figure(3); clf; hold on;
for i = 1:length(centiles)
    plot(timesteps, percentile(error1,centiles(i)),...
        'Color',color(i,:));
end
xlabel('time (s)');
ylabel('trajectory error (m)');
title('Error percentiles for ideal clustering');
legend('50%','25%','10%','5%');

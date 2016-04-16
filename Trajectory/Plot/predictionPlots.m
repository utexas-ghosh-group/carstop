clear;
%% 4/14/16
dataName = 's3sb3_raw';
methodName = 'CV';
overallTitle = 'test plot';
plotType = 'trajectory';
% types: trajectory, error, mean, percentile, color histogram


% inputs: truth (3d matrix, traj X time X [x,y])
%         prediction (same format)
truth = load(cat(2,'truth_',dataName,'.mat'),'timeMatrix');
truth = truth.timeMatrix;
maneuvers = load(cat(2,'truth_',dataName,'.mat'),'contextMatrix');
maneuvers = maneuvers.contextMatrix;
prediction = load(cat(2,'trajectory_',dataName,'_',methodName,'.mat'),...
                    'predictedTrajectory');
prediction = prediction.predictedTrajectory;
                
ntrajects = size(truth,1);
ntimesteps = size(truth,2);
timesteps = (1:ntimesteps) * 0.1;
percentile = @(vals, fraction) [zeros(1,floor(size(vals,1)*fraction)-1), ...
    1, zeros(1,ceil(size(vals,1)*(1-fraction)))] * ...
    sort(vals,1,'descend');

testGroups = ones(ntrajects,1);
titles = {'Turn','Straight','Lane change'};

y_max = 0;
for groupi = 1:size(testGroups,2)
    group = find(testGroups(:,groupi))';
    error = ((truth(group,:,1) - prediction(group,:,1)).^2 +...
             (truth(group,:,2) - prediction(group,:,2)).^2 ).^.5;
    centiles = [.5 .25 .1];
    for i = 1:length(centiles)
        temp = max(percentile(error, centiles(i)));
        if y_max < temp
            y_max = temp;
        end
    end
end
y_max = 1.1*y_max;
x_max = 1.1*max(timesteps);

figure(1); clf; hold on;

for groupi = 1:size(testGroups,2)
         
    group = find(testGroups(:,groupi))';
    error = ((truth(group,:,1) - prediction(group,:,1)).^2 +...
             (truth(group,:,2) - prediction(group,:,2)).^2 ).^.5;
 
    subplot(1, size(testGroups,2), groupi);
    
    %title( titles{groupi});
    hold on;
    
    switch plotType
    case 'trajectory'
        for obs = group
            pA = squeeze(truth(obs,:,:));
            pB = squeeze(prediction(obs,:,:));
            plot(pA(:,1), pA(:,2),'b.',pA(1,1),pA(1,2),'bo',...
                 pB(:,1), pB(:,2),'r.',pB(1,1),pB(1,2),'ro');
        end
        xlabel('x (m)'); ylabel('y (m)');
        
    case 'error'
        for obs = 1:size(error,1)
            plot(timesteps, error(obs,:));
        end
        xlabel('time (s)');
        ylabel('trajectory error (m)');
        
    case 'mean'
        grid on;
        plot(timesteps,mean(error));
        xlabel('time (s)');
        ylabel('mean trajectory error (m)');
        
    case 'percentile'
        grid on;
        centiles = [.5 .25 .1];
        color = parula(length(centiles)+1);
        for i = 1:length(centiles)
            plot(timesteps, percentile(error, centiles(i)),...
                'Color',color(i,:));
        end
        xlabel('time (s)');
        ylabel('percentile trajectory error (m)');
        axis([0 x_max 0 y_max]);
        legend('50%','25%','10%','Location','northwest');
            
    case 'color histogram'
        edges = [0:2:20,inf];
        hist2d = zeros(ntimesteps, length(edges));
        for tstep = 1:ntimesteps
            hist2d(tstep,:) = histc(error(:,tstep),edges);
        end
        hist2d = hist2d.^.5; % evens out color scale
        colormap('default');
        imagesc(timesteps, edges(1:length(edges)-1)+1.5,...
                hist2d(:,1:length(edges)-1)');
        %colorbar();
        xlabel('time (s)');
        ylabel('trajectory error (m)');
    end
end
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],...
    'Box','off','Visible','off','Units','normalized','clipping','off');
text(0.5,1, cat(2,'\bf ',overallTitle),...
    'HorizontalAlignment','center','VerticalAlignment', 'top');
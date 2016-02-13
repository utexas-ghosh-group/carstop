load testGroups_s2nb2.mat;

% inputs: truth (3d matrix, traj X time X [x,y])
%         prediction (same format)

overallTitle = 'Map-based Trajectory Prediction';
plotType = 'trajectory';
% types: trajectory, error, mean, percentile, color histogram

todo = [3,5,9,19,22,36,113,182,215,332,423,554];
colors = hsv(4);
testGroups = testGroups(todo,:);
truth = truth(todo,:,:);
prediction = prediction(todo,:,:);

ntrajects = size(truth,1);
ntimesteps = size(truth,2);
timesteps = (1:ntimesteps) * 0.1;
percentile = @(vals, fraction) [zeros(1,floor(size(vals,1)*fraction)-1), ...
    1, zeros(1,ceil(size(vals,1)*(1-fraction)))] * ...
    sort(vals,1,'descend');

titles = {'Turn','Straight','Lane change'};

if strcmp(plotType,'percentile')
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
    y_max = 1.05*y_max;
    x_max = max(timesteps);
end

figure(1); clf; hold on;
for groupi = 1:size(testGroups,2)
         
    group = find(testGroups(:,groupi))';
    error = ((truth(group,:,1) - prediction(group,:,1)).^2 +...
             (truth(group,:,2) - prediction(group,:,2)).^2 ).^.5;
 
    subplot(1, size(testGroups,2), groupi);
    axis([-33,17,-70,30]);
    title( titles{groupi} );
    hold on;
    
    switch plotType
    case 'trajectory'
        count=1;
        for obs = group
            pA = squeeze(truth(obs,:,:));
            pB = squeeze(prediction(obs,:,:));
            if groupi==2
                pB(:,1) = pB(:,1)+1;
            end
            %plot(pA(:,1), pA(:,2),'b.',pA(1,1),pA(1,2),'bo',...
            %     pB(:,1), pB(:,2),'r.',pB(1,1),pB(1,2),'ro');
            plot(pA(:,1),pA(:,2),'LineStyle','-','Color',colors(count,:));
            plot(pB(:,1),pB(:,2),'LineStyle','--','Color',colors(count,:));
            count=count+1;
        end
        if groupi==1
            ylabel('North-South (m)');
        end
        if groupi==2
            xlabel('East-West (m)');
        end
        
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
        if groupi==1
            ylabel('trajectory error (m)');
            legend('50%','25%','10%','Location','northwest');
        end
        if groupi == 2
            xlabel('time (s)');
        end
        axis([0 x_max 0 y_max]);
            
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
        if groupi==1
            xlabel('time (s)');
            ylabel('trajectory error (m)');
        end
    end
end
ha = axes('Position',[0 0 1 1],'Xlim',[0 1],'Ylim',[0 1],...
    'Box','off','Visible','off','Units','normalized','clipping','off');
text(0.5,1, cat(2,'\bf ',overallTitle),...
    'HorizontalAlignment','center','VerticalAlignment', 'top');


% figure(1);clf;hold on;
% zz = zeros(length(centroids),1);
% for cc = [21:25]
%     cd = centroids(cc).x;
%     plot(cd(:,1),cd(:,2));
%     zz(cc) = cd(1,2);
% end

%     figure(1); clf; hold on; grid on;
%     axis([-40 40 -80 80]);
%     for i = 1:7
%     plot(MPs(i).aftmean(:,1)+.5*MPs(i).aftvar(:,1),MPs(i).aftmean(:,2)+.5*MPs(i).aftvar(:,2),'LineWidth',4);
%     end
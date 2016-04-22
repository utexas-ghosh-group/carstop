clear;
datafile = 'rand1k_roadCoord_CVroad';
truthfile = 'rand1k';
save = 1;

prediction = load(cat(2,'trajectory_',datafile,'.mat'));
predictInclude = prediction.includedFromSegment;
prediction = prediction.predictedTrajectory;


%% euclidean errors
truth = load(cat(2,'truth_',truthfile,'_raw.mat'));
maneuvers = truth.contextMatrix;
trueInclude = truth.includedFromSegment;
truth = truth.timeMatrix;

% make sure both datasets cover the same vehicles
trueInclusion = predictInclude(trueInclude>0)>0;
truth = truth(trueInclusion,:,:);
maneuvers = maneuvers(trueInclusion,:);
predInclusion = trueInclude(predictInclude > 0)>0;
prediction = prediction(predInclusion,:,:);

errors = ((truth(:,:,1)-prediction(:,:,1)).^2+...
            (truth(:,:,1)-prediction(:,:,1)).^2 ).^.5;

errorTable = zeros(7,3);
for maneuver = 0:5
    group = errors(maneuvers==maneuver,:);
    errorTable(maneuver+2,1) = mean(group(:,10));
    errorTable(maneuver+2,2) = mean(group(:,30));
    errorTable(maneuver+2,3) = mean(group(:,50));
end
    
errorTable(1,1) = mean(errors(:,10));
errorTable(1,2) = mean(errors(:,30));
errorTable(1,3) = mean(errors(:,50));
errorTable

%% longitudinal errors
% truth = load(cat(2,'truth_',truthfile,'.mat'),'timeMatrix');
% truth = truth.timeMatrix;
% maneuvers = load(cat(2,'truth_',dataName,'_roadCoord.mat'),'contextMatrix');
% maneuvers = maneuvers.contextMatrix;
% 
% errors = abs(truth(:,:,1)-prediction(:,:,1));
% 
% errorTable = zeros(7,3);
% for maneuver = 0:5
%     group = errors(maneuvers==maneuver,:);
%     errorTable(maneuver+2,1) = mean(group(:,10));
%     errorTable(maneuver+2,2) = mean(group(:,30));
%     errorTable(maneuver+2,3) = mean(group(:,50));
% end
%     
% errorTable(1,1) = mean(errors(:,10));
% errorTable(1,2) = mean(errors(:,30));
% errorTable(1,3) = mean(errors(:,50));
% errorTable

%%
if save > 0
    csvwrite(cat(2, datafile,'_results.csv'),errorTable);
end
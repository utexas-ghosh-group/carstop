% 4/23/16
clear;
datafile = 'rand2k_av1';
truthfile = 'rand2k';
saveresults = 1;

prediction = load(cat(2,'trajectory_',datafile,'.mat'));
predictInclude = prediction.includedFromSegment;
truth = prediction.truth;
prediction = prediction.predictedTrajectory;


%% euclidean errors
struth = load(cat(2,'truth_',truthfile,'_roadCoord.mat'));
maneuvers = struth.contextMatrix;
trueInclude = struth.includedFromSegment;
%truth = truth.timeMatrix;

% make sure both datasets cover the same vehicles
trueInclusion = predictInclude(trueInclude>0)>0;
maneuvers = maneuvers(trueInclusion,:);
predInclusion = trueInclude(predictInclude > 0)>0;
prediction = prediction(predInclusion,:,:);
truth = truth(predInclusion,:,:);

% for now, get rid of broken estimates
broken = any(prediction(:,:,2)==0,2);
truth = truth(broken==0,:,:);
prediction = prediction(broken==0,:,:);
maneuvers = maneuvers(broken==0,:);

errors = ((truth(:,:,1)-prediction(:,:,1)).^2+...
            (truth(:,:,1)-prediction(:,:,1)).^2 ).^.5;

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

errorTable = zeros(5,3);
errorTable(1,1) = mean(errors(:,10));
errorTable(1,2) = mean(errors(:,30));
errorTable(1,3) = mean(errors(:,50));
group = errors(maneuvers==0,:);
errorTable(2,1) = mean(group(:,10));
errorTable(2,2) = mean(group(:,30));
errorTable(2,3) = mean(group(:,50));
group = errors(maneuvers==3,:);
errorTable(3,1) = mean(group(:,10));
errorTable(3,2) = mean(group(:,30));
errorTable(3,3) = mean(group(:,50));
group = errors((maneuvers==2)+(maneuvers==4)>0,:);
errorTable(4,1) = mean(group(:,10));
errorTable(4,2) = mean(group(:,30));
errorTable(4,3) = mean(group(:,50));
group = errors((maneuvers==1)+(maneuvers==5)>0,:);
errorTable(5,1) = mean(group(:,10));
errorTable(5,2) = mean(group(:,30));
errorTable(5,3) = mean(group(:,50));
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
if saveresults > 0
    csvwrite(cat(2, 'm', datafile,'_results.csv'),errorTable);
end
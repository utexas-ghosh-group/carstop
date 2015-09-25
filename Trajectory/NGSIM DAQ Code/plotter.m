%% local data from section
load('data_s2_nb.mat');

selectData = (vehicleMatrix(:,2) == 102).*(vehicleMatrix(:,3)==208).*(vehicleMatrix(:,1)==3);

timeMatrix = timeMatrix(selectData >0,:,:);
timeRange = 1:size(timeMatrix,2);
colors = hsv(12);

figure(1); clf;
hold on; grid on;
for i = 1:size(timeMatrix,1)
    %if(vehicleMatrix(i,3) == 211)
    plot(timeMatrix(i,timeRange,1),timeMatrix(i,timeRange,2),...
        'Color',colors(vehicleMatrix(i,3)-200,:));
    plot(timeMatrix(i,1:4,1),timeMatrix(i,1:4,2),'k-');
    %end
end

prevA = [0,60.5];
prevB = [16,59];
linX = linspace(prevA(1),prevB(1),20);
linY = linspace(prevA(2),prevB(2),20);
plot(linX, linY, 'k-.');

pointA = [1,90];
pointB = [17,90];
distA = norm(pointA - prevA, 2)
distB = norm(pointB - prevB, 2)
linX = linspace(pointA(1),pointB(1),20);
linY = linspace(pointA(2),pointB(2),20);
plot(linX, linY, 'k-.');

%% global data from segment
load('segment_s2nb_2.mat');
colors = hsv(12);
timeBefore = 1:70;
timeAfter = 72:141;

figure(1); clf;
hold on; grid on;
for i = 1:size(vehicleMatrix,1)
    %if group2(i)
    plot(timeMatrix(i,timeAfter,1),timeMatrix(i,timeAfter,2))%,...
        %'Color',colors(vehicleMatrix(i,2)-200,:));
    %end
end
%scatter(timeMatrix(:,71,1), timeMatrix(:,71,2),'k.');
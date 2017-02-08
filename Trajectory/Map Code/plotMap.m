load('roads_all.mat');
figure(1); clf; hold on;
% plot roads
for i = 7:11%length(roads)
    %xx = roads(i).segments;
    points = interpolate(roads(i).segments , 1);
    plot(points(:,1),points(:,2));
end

% plot vehicles
%load('data_rand1k_raw.mat');
for i = 1:size(timeMatrix,1)
    %partsToPlot = (timeMatrix(i,:,3) >= 1).*(timeMatrix(i,:,3) <= 6) > 0;
    partsToPlot = [];
    if contextMatrix(i) == 1
        partsToPlot = 1:50;
    end
    if any(partsToPlot)
        plot(timeMatrix(i,partsToPlot,1),timeMatrix(i,partsToPlot,2),'k.');
    end
    %firstpoint = find((timeMatrix(i,:,3)==4)+(timeMatrix(i,:,3)==5)>0, 1);
    %plot(timeMatrix(i,firstpoint,1),timeMatrix(i,firstpoint,2),'ko');
%     plot(guess(i,:,1), guess(i,:,2),'r.');
end
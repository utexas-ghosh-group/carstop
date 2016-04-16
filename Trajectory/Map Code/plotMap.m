load('roads_all.mat');
load('segment_s2nb_2.mat');
figure(1); clf; hold on;
% plot roads
for i = 1:6%length(roads)
    xx = roads(i).segments;
    points = interpolate(roads(i).segments , 1);
    plot(points(:,1),points(:,2));
end
% plot vehicles
for i = find(aa>15)%1:%size(timeMatrix,1)
    plot(timeMatrix(i,:,1),timeMatrix(i,:,2),'k.');
    firstpoint = find((timeMatrix(i,:,3)==4)+(timeMatrix(i,:,3)==5)>0, 1);
    plot(timeMatrix(i,firstpoint,1),timeMatrix(i,firstpoint,2),'ko');
%     plot(guess(i,:,1), guess(i,:,2),'r.');
end
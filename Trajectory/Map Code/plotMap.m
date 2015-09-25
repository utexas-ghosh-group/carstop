load('roads_all.mat');
load('segment_s2nb_2.mat');
figure(1); clf; hold on;
for i = [9,40]%1:length(roads)
    xx = roads(i).segments;
    points = interpolate(roads(i).segments , 1);
    plot(points(:,1),points(:,2));
end
for i = [70 208 209 214 252 411 417 425 453 456 461 471]%1:%size(timeMatrix,1)
    plot(timeMatrix(i,:,1),timeMatrix(i,:,2),'k.');
    plot(guess(i,:,1), guess(i,:,2),'r.');
end
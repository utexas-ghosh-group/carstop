%4/13/16
load('roads_all.mat')
existingRoads = cell(0,1);
adjacencies = 1:length(roads);
for i = 1:length(roads)
   elements = strsplit(roads(i).name, '_');
   if length(elements) <= 2 % not a transition
       road = cell2mat(elements(1));
   else
       if strcmp(cell2mat(elements(2)), 'X')
           secondRoad = cell2mat(elements(3));
       elseif strcmp(cell2mat(elements(3)), 'X')
           secondRoad = cell2mat(elements(4));
       end
       road = cat(2,cell2mat(elements(1)),'_X_',secondRoad);
   end
   
   found = 0;
   for roadidx = 1:length(existingRoads)
       if strcmp(cell2mat(existingRoads(roadidx)), road)
           found = 1;
           adjacencies(i) = roadidx;
           break;
       end
   end
   if ~found
       existingRoads = cat(1, existingRoads, road);
       adjacencies(i) = length(existingRoads);
   end
end
save('roads_all.mat','roads','transitions','adjacencies')
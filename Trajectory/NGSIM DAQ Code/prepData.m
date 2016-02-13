temp = importdata('trajectories-0830am-0845am.txt');
temp2 = importdata('trajectories-0845am-0900am.txt');

temp = temp(:,[1:8,14:20,21,23]);
temp2 = temp2(:,[1:8,14:20,21,23]);

% offset the vehicle and frame ID's of dataset 2 so they don't overlap
temp2(:,1) = temp2(:,1) + 1500;
temp2(:,2) = temp2(:,2) + 12000;

temp = cat(1,temp,temp2);
clear temp2;

% offset time
temp(:,4) = temp(:,4) - 1.11893*10^12;

% convert to meters
temp(:,5:8) = temp(:,5:8) * .3048;
% offset the global position
temp(:,7) = temp(:,7) - 1966000;
temp(:,8) = temp(:,8) - 570000;

% front vehicle stuff
temp(:,17) = temp(:,17) * .3048;

dlmwrite('preppedData2.csv',temp,'precision',7,'delimiter',',');

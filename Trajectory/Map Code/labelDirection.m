function direction = labelDirection(roadNumsBefore, roadNumsAfter)
% 0 = no movement
% 1 = left
% 2 = left lane change
% 3 = straight
% 4 = right lane change
% 5 = right
% <0 = error
% 4/15/16

    global roads; global transitions; global adjacencies;
    nroads = length(roads);
    
    numBefore  = DiscreteHist(...
        roadNumsBefore(length(roadNumsBefore)-5:length(roadNumsBefore))');
    numAfter = DiscreteHist(...
        roadNumsAfter(length(roadNumsAfter)-5:length(roadNumsAfter)));
    
    numBefore = numBefore(1); % already sorted descending
    numAfter = numAfter(1);
    
    if numBefore == numAfter
        direction = 0;
        return;
    end
    
    nameBefore = roads(numBefore).name;

    leftRoadNames = cell(0,1);
    %straightRoadNames = cell(0,1);
    rightRoadNames = cell(0,1);
    if strncmp(nameBefore, 's3sb', 4)
        leftRoadNames = cat(1, leftRoadNames, '211', 's3sb_X_211');
        %straightRoadNames = cat(1, straightRoadNames, 's2sb', 's3sb_X_s2sb');
        rightRoadNames = cat(1, rightRoadNames, '203', 's3sb_X_203');
    end
    
    % add adjacents
    leftLCnums = zeros(nroads);
    leftLCnames = cell(0,1);
    rightLCnums = zeros(nroads);
    laneNumBefore = laneNum(nameBefore);
    for ridx = 1:nroads
        if ridx ~= numBefore && adjacencies(ridx) == adjacencies(numBefore)
            laneNumHere = laneNum(roads(ridx).name);
            if laneNumBefore > laneNumHere
                leftLCnums(ridx) = 1;
                leftLCnames = cat(1,leftLCnames,roads(ridx).name);
            else
                rightLCnums(ridx) = 1;
            end
        end
    end
    
    % add transitions
    index = 1:nroads;
    allAdjacents = (leftLCnums + rightLCnums) > 0;
    allAdjacents(numBefore) = 1;
    allTransitions = index(any(transitions(allAdjacents,:),1));
    leftTnums = [];
    rightTnums = [];
    straightnums = [];
    for transNum = allTransitions
        transName = simplifyRoad(roads(transNum).name);
        if any(strcmp(leftRoadNames, transName))
            leftTnums = cat(1, leftTnums, transNum);
        elseif any(strcmp(rightRoadNames, transName))
            rightTnums = cat(1, rightTnums, transNum);
        elseif any(transitions(leftLCnums>0, transNum))
            leftLCnums(transNum) = 1;
        elseif any(transitions(rightLCnums>0, transNum))
            rightLCnums(transNum) = 1;
        else
            straightnums = cat(1, straightnums, transNum);
        end
    end
    leftLCnums =  index(leftLCnums>0);
    rightLCnums = index(rightLCnums>0);
    
%     % add all destination lanes
%     leftLaneNames = cell(0,1);
%     leftLaneNums = [];
%     straightLaneNames = cell(0,1);
%     straightLaneNums = [];
%     rightLaneNames = cell(0,1);
%     rightLaneNums = [];
%     for ridx = 1:nroads
%         laneName = roads(ridx).name;
%         for idx = 1:length(leftRoadNames)
%             leftRoadName = cell2mat(leftRoadNames(idx));
%             if strncmp(leftRoadName, laneName, length(leftRoadName))
%                 leftLaneNames = cat(1, leftLaneNames, laneName);
%                 leftLaneNums = cat(1, leftLaneNums, ridx);
%             end
%         end
%         for idx = 1:length(straightRoadNames)
%             straightRoadName = cell2mat(straightRoadNames(idx));
%             if strncmp(straightRoadName, laneName, length(straightRoadName))
%                 straightLaneNames = cat(1, straightLaneNames, laneName);
%                 straightLaneNums = cat(1, straightLaneNums, ridx);
%             end
%         end
%         for idx = 1:length(rightRoadNames)
%             rightRoadName = cell2mat(rightRoadNames(idx));
%             if strncmp(rightRoadName, laneName, length(rightRoadName))
%                 rightLaneNames = cat(1, rightLaneNames, laneName);
%                 rightLaneNums = cat(1, rightLaneNums, ridx);
%             end
%         end
%     end
%     
%     % add transitions
%     for ridx = 1:length(leftLaneNums)
%         if transitions(numBefore, leftLaneNums(ridx))
%             newname = cat(2, nameBefore, '_X_', ...
%                             cell2mat(leftLaneNames(ridx)));
%             leftLaneNames = cat(1, leftLaneNames, newname);
%         end
%     end
%     for ridx = 1:length(straightLaneNums)
%         if transitions(numBefore, straightLaneNums(ridx))
%             newname = cat(2, nameBefore, '_X_', ...
%                             cell2mat(straightLaneNames(ridx)));
%             straightLaneNames = cat(1, straightLaneNames, newname);
%         end
%     end
%     for ridx = 1:length(rightLaneNums)
%         if transitions(numBefore, rightLaneNums(ridx))
%             newname = cat(2, nameBefore, '_X_', ...
%                             cell2mat(rightLaneNames(ridx)));
%             rightLaneNames = cat(1, rightLaneNames, newname);
%         end
%     end
    
    % now we can actually check where the car went
%     leftTnums
%     straightnums
%     rightTnums
%     leftLCnums
%     rightLCnums
    if any(leftTnums == numAfter)
        direction = 1;
    elseif any(straightnums == numAfter)
        direction = 3;
    elseif any(rightTnums == numAfter)
        direction = 5;
    elseif any(leftLCnums == numAfter)
        direction = 2;
    elseif any(rightLCnums == numAfter)
        direction = 4;
    else
        direction = -1;
    end
end


function num = laneNum(laneName)
    eles = strsplit(laneName,'_');
    num = str2double(cell2mat(eles(2)));
    if num > 10 && num < 20
        num = num-20;
    end
end


function name = simplifyRoad(lanename)
    eles = strsplit(lanename,'_');
    if length(eles) > 2
        if strcmp(cell2mat(eles(2)),'X')
            name = cat(2, cell2mat(eles(1)),'_X_',cell2mat(eles(3)));
        else
            name = cat(2, cell2mat(eles(1)),'_X_',cell2mat(eles(4)));
        end
    else
        name = cell2mat(eles(1));
    end
end
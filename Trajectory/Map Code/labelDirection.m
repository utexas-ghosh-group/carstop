function direction = labelDirection(roadNumsBefore, roadNumsAfter)
% 0 = no movement
% 1 = left
% 2 = left lane change
% 3 = straight
% 4 = right lane change
% 5 = right
% <0 = error
% 4/21/16

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
    rightRoadNames = cell(0,1);
    if strncmp(nameBefore, 's3sb', 4)
        leftRoadNames = cat(1, leftRoadNames, '203', 's3sb_X_203');
        rightRoadNames = cat(1, rightRoadNames, '211', 's3sb_X_211');
    elseif strncmp(nameBefore, 's2nb', 4)
        leftRoadNames = cat(1, leftRoadNames, '211', 's2nb_X_211');
        rightRoadNames = cat(1, rightRoadNames, '203', 's2nb_X_203');
    elseif strncmp(nameBefore, 's3nb', 4)
        leftRoadNames = cat(1, leftRoadNames, '210', 's3nb_X_210');
        rightRoadNames = cat(1, rightRoadNames, '205', 's3nb_X_205');
    elseif strncmp(nameBefore, 's4sb', 4)
        leftRoadNames = cat(1, leftRoadNames, '205', 's4sb_X_205');
        rightRoadNames = cat(1, rightRoadNames, '210', 's4sb_X_210');
    elseif strncmp(nameBefore, 's4nb', 4)
        leftRoadNames = cat(1, leftRoadNames, '209', 's4nb_X_209');
        rightRoadNames = cat(1, rightRoadNames, '207', 's4nb_X_207');
    elseif strncmp(nameBefore, '108', 4)
        leftRoadNames = cat(1, leftRoadNames, '207', '108_X_207');
        rightRoadNames = cat(1, rightRoadNames, '209', '108_X_209');
    elseif strncmp(nameBefore, '109', 4)
        leftRoadNames = cat(1, leftRoadNames, '208', '109_X_208');
        rightRoadNames = cat(1, rightRoadNames, 's4sb', '109_X_s4sb');
    elseif strncmp(nameBefore, '107', 4)
        leftRoadNames = cat(1, leftRoadNames, 's4sb', '107_X_s4sb');
        rightRoadNames = cat(1, rightRoadNames, '208', '107_X_208');
    elseif strncmp(nameBefore, '110', 4)
        leftRoadNames = cat(1, leftRoadNames, 's4nb', '110_X_s4nb');
        rightRoadNames = cat(1, rightRoadNames, 's3sb', '110_X_s3sb');
    elseif strncmp(nameBefore, '105', 4)
        leftRoadNames = cat(1, leftRoadNames, 's3sb', '105_X_s3sb');
        rightRoadNames = cat(1, rightRoadNames, 's4nb', '105_X_s4nb');
    elseif strncmp(nameBefore, '111', 4)
        leftRoadNames = cat(1, leftRoadNames, 's3nb', '111_X_s3nb');
        rightRoadNames = cat(1, rightRoadNames, 's2sb', '111_X_s2sb');
    elseif strncmp(nameBefore, '103', 4)
        leftRoadNames = cat(1, leftRoadNames, 's2sb', '103_X_s2sb');
        rightRoadNames = cat(1, rightRoadNames, 's3nb', '103_X_s3nb');
    elseif strncmp(nameBefore, '102', 4)
        leftRoadNames = cat(1, leftRoadNames, '201', '102_X_201');
        rightRoadNames = cat(1, rightRoadNames, 's2nb', '102_X_s2nb');
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
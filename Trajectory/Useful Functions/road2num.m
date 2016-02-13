function num = road2num(section, direction, lane, prev_section, prev_direction,...
			prev_lane, next_section, next_direction, next_lane, roadNames)
    if (nargin == 9)
        roadNames = importdata('roadList.txt');
    end

    dir2char = @(dir) 'e'*(dir==1)+'n'*(dir==2)+'w'*(dir==3)+'s'*(dir==4);
    
    if section > 0
        if section < 100 % main road
            dirName = dir2char(direction);
            name = cat(2,'s',num2str(section),dirName,'b_',...
                    num2str(lane));
        else % entry
            name = cat(2,num2str(section),'_',num2str(lane));
        end
        
        name = rename(name);
    
    else
        if (prev_section < 100) % main road
            dirName = dir2char(prev_direction);
            prev_name = cat(2,'s',num2str(prev_section),dirName,'b_',...
                    num2str(prev_lane));
        else % entry
            prev_name = cat(2,num2str(prev_section),'_',num2str(prev_lane));
        end
        prev_name = rename(prev_name);
        
        if (next_section < 100) % main road
            dirName = dir2char(next_direction);
            next_name = cat(2,'s',num2str(next_section),dirName,'b_',...
                    num2str(next_lane));
        else % entry
            next_name = cat(2,num2str(next_section),'_',num2str(next_lane));
        end
        next_name = rename(next_name);
        
        name = cat(2, prev_name, '_X_', next_name);
    end

    %num = find(strcmp(roads,name));
    num = 0;
    for k = 1:length(roadNames)
        if strcmp(roadNames(k), name)
            num = k;
        end
    end
    
    if (num == 0) % road not found, could be changing lanes
        if (section == 0) && (prev_lane ~= next_lane)
            % try the future lane and hope it is treated like a lane change
            num = road2num(section, direction, lane, prev_section,...
                           prev_direction, next_lane, next_section,...
                           next_direction, next_lane, roadNames);
        end
    end
    if num == 0 % this road is still not found in the list
        unknown_road = name % print name
    end
end

function newname = rename(road)
    % one lane roads
    if strcmp(road(1:3), '204')
        newname = '204_1';
    elseif strcmp(road(1:3), '106')
        newname = '106_1';
    elseif strcmp(road(1:3), '206')
        newname = '206_1';
    elseif strcmp(road(1:3), '104')
        newname = '104_1';
    elseif strcmp(road(1:3), '209')
        newname = '209_1';
    % roads that are named twice, as section and as entrance/exit
    elseif strcmp(road(1:4), 's5sb')
        newname = cat(2,'108',road(5:end));
    elseif strcmp(road(1:4), 's5nb')
        newname = cat(2,'208',road(5:end));
    elseif strcmp(road(1:4), 's1sb')
        newname = cat(2,'201',road(5:end));
    elseif strcmp(road(1:4), 's1nb')
        newname = cat(2,'101',road(5:end));
    else
        newname = road;
    end
end

function [section, direction, lane, prev_section, ...
		prev_direction, prev_lane, next_section, ...
		next_direction, next_lane] = num2road(num, roadNames)

    section = 0;
    direction = 0;
    lane = 0;
    prev_section = 0;
    prev_direction = 0;
    prev_lane = 0;
    next_section = 0;
    next_direction = 0;
    next_lane = 0;
    
    if (nargin == 1)
        roadNames = importdata('roadList.txt');
    end

    roadName = cell2mat(roadNames(num));
    splitRoadName = strsplit(roadName,'_X_');
    
    if (length(splitRoadName) == 1) % in section
        if strcmp(roadName(1), 's') % section
            section = str2double(roadName(2));
            if strcmp(roadName(3), 's')
                direction = 4;
            else
                direction = 2;
            end
            segments = strsplit(roadName, '_');
            lane = str2double( segments(end) );
        else   % entrance or exit
            segments = strsplit(roadName, '_');
            section = str2double(segments(1));
            if length(segments)>1
                lane = str2double(segments(2));
            end
        end
        
    else
        prev_roadName = cell2mat(splitRoadName(1));
        if strcmp(prev_roadName(1), 's') % section
            prev_section = str2double(prev_roadName(2));
            if strcmp(prev_roadName(3), 's')
                prev_direction = 4;
            else
                prev_direction = 2;
            end
            segments = strsplit(prev_roadName, '_');
            prev_lane = str2double( segments(end) );
        else   % entrance
            segments = strsplit(prev_roadName, '_');
            prev_section = str2double(segments(1));
            if length(segments)>1
                prev_lane = str2double(segments(2));
            end
        end

        next_roadName = cell2mat(splitRoadName(2));
        if strcmp(next_roadName(1), 's') % section
            next_section = str2double(next_roadName(2));
            if strcmp(next_roadName(3), 's')
                next_direction = 4;
            else
                next_direction = 2;
            end
            segments = strsplit(next_roadName, '_');
            next_lane = str2double( segments(end) );
        else   % exit
            segments = strsplit(next_roadName, '_');
            next_section = str2double(segments(1));
            if length(segments)>1
                next_lane = str2double(segments(2));
            end
        end
    end
end

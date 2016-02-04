fid = fopen('I2_2_16.kml','r');
line = fgetl(fid);
names = {'names'};
coords = {'coords','',''};
readyForName=0;
emerg=0;
while emerg < 100000  && ~all(line == -1)
    line = line(~isspace(line));
    if strcmp(line, '<Placemark>')
        readyForName = 1;
    end
    if strcmp(line, '</Placemark>')
        readyForName = 0;
    end
    if strncmp(line,'<name>',6) && readyForName
        names = cat(1, names, line(7:length(line)-7));
    end
    if strncmp(line,'<coordinates>',13) && readyForName
        numbers = line(14:length(line)-14);
        coords = cat(1, coords, strsplit(numbers,','));
    end
    
    line = fgetl(fid);
    emerg = emerg + 1;
end
fclose(fid);

fid = fopen('unconverted.csv','w');
for row = 2:size(coords,1)
    fprintf(fid, '%s %s %s\n',coords{row,:});
end
fclose(fid);

%%
names = names(2:size(names,1));
save('roadnames.mat','names');
% fid = fopen('roads2coords.txt','w');
% for row = 2:size(names,1)
%     fprintf(fid, '%s\n',names{row});
% end
% fclose(fid);
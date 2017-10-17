[~,len] = size(pcObj);

pcGround = cell(1,len);
for i = 1:len
%     pcCount = pcObj{i}.Count;
    i
%     locationNew = [0];
%     locationCount = 0;
%     for j = 1:pcCount
%         if(pcObj{i}.Location(j,3) <= 0)
%             locationCount = locationCount + 1;
%             locationNew(locationCount) = j;
%         end
%     end
    [locationNew,~] = findNeighborsInRadius(pcObj{i},[0,0,0],2);
    pcGround{i} = select(pcObj{i},locationNew);
end
pcshow(pcGround{1});
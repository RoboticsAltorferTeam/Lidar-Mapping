% [~,len] = size(pcObj);
[len1,~] = size(lidarData);
[len2,~] = size(odomData);

odomIndex = 1 : ((len2 - 1)/len1): len2;
odomIndex = floor(odomIndex);
[~,len2] = size(odomIndex);
odomData = odomData(odomIndex);
len = min([len1,len2]);

initPos = [odomData{1}.Pose.Pose.Position.X,...
    odomData{1}.Pose.Pose.Position.Y,...
    odomData{1}.Pose.Pose.Position.Z];
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
%     [locationNew,~] = findNeighborsInRadius(pcObj{i},[0,0,0],2);
%     pcGround{i} = select(pcObj{i},locationNew);

%     Select only the first ring of Lidar Data
    ring0 = categorical(lidarData{i}.readField('ring'));
    ringIndices = find(ismember(ring0,{'0'}));
    
%     Get the location Data
    curPos = [odomData{i}.Pose.Pose.Position.X,...
        odomData{i}.Pose.Pose.Position.Y,...
        odomData{i}.Pose.Pose.Position.Z];
    locationData = readXYZ(lidarData{i});
%     locationData = locationData(ringIndices,:);
    locationDataPos = locationData + curPos;
%     pcGround{i} = select(pcObj{i},ringIndices);

%     Build the Pointcloud MATLAB class object
    intensityData = readField(lidarData{i},'intensity');
%     intensityData = intensityData(ringIndices);
    pcGround{i} = pointCloud(locationDataPos, 'Intensity', intensityData);
    
    if(i == 2)
        pcOut = pcmerge(pcGround{1}, pcGround{2}, 0.00001);
%         locationAll = [locationAll;locationDataPos];
    elseif(i == 1)
        locationAll = locationDataPos;
    elseif(i > 2)
        pcOut = pcmerge(pcOut, pcGround{i}, 0.00001);
        if(i == len)
            locationAll = [locationAll;locationDataPos];
        end
    end
end
pcMergedAll = pointCloud(locationAll);
figure,pcshow(pcMergedAll);
hold on;
plot3(initPos(1),initPos(2),initPos(3),'go','MarkerSize',10,'MarkerFaceColor','g');
plot3(curPos(1),curPos(2),curPos(3),'ro','MarkerSize',10,'MarkerFaceColor','r');
hold off;
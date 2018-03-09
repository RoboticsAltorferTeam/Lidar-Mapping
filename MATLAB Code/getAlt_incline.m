% [~,len] = size(pcObj);

useRings = 1;   %Use only some of the rings (set 1). Use all rings (set 0)
lidarHt = 0;
% lidarZoffset = -0.4017;
lidarZoffset = -0.406;
% gridSize = 0.01;
% odomData = odomFilData;

if(~exist('lidarData','var'))
    len1 = numMessages;
else
    [len1,~] = size(lidarData);
end
[len2,~] = size(odomData);

positionAll = zeros(len2,3,'double');
for i = 1:len2
    positionAll(i,:) = [odomData{i}.Pose.Pose.Position.X,...
        odomData{i}.Pose.Pose.Position.Y,...
        odomData{i}.Pose.Pose.Position.Z];
end

if(len2 > len1 + 1)
    odomIndex = 1 : ((len2 - 1)/len1): len2;
    odomIndex = floor(odomIndex);
    [~,len2] = size(odomIndex);
    odomData = odomData(odomIndex);
    len = min([len1,len2]);
elseif(len1 > len2)
    len = len2;
else
    len = len1;
end
% len = 300;

pcGround = cell(1,len);
lidarDataTemp = cell(1,1);
timeLidar = zeros(len,1,'double');
timeOdom = zeros(len,1,'double');
seqLidar = zeros(len,1,'double');
seqOdom = zeros(len,1,'double');
positionAll = zeros(len,3,'double');
clear locationAll;
clear locationAllR;
for i = 1:len
    timeOdom(i) = odomData{i}.Header.Stamp.Sec +...
        (odomData{i}.Header.Stamp.Nsec * 1e-9);
    seqOdom(i) = odomData{i}.Header.Seq;
    
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

    if(~exist('lidarData','var'))
        lidarDataTemp = readMessages(veloBag,i);
    else
        lidarDataTemp{1} = lidarData{i};
    end

    timeLidar(i) = lidarDataTemp{1}.Header.Stamp.Sec +...
        (lidarDataTemp{1}.Header.Stamp.Nsec * 1e-9);
    seqLidar(i) = lidarDataTemp{1}.Header.Seq;
%     Select only the first ring of Lidar Data
    ring0 = categorical(lidarDataTemp{1}.readField('ring'));
%     ringIndices = find(ismember(ring0,{'0','1','2','3'}));
    ringIndices = find(ismember(ring0,{'0'}));
    
    locationData = readXYZ(lidarDataTemp{1});
    if(useRings == 1)
        locationData = locationData(ringIndices,:);
    end
    
    %Incline code
    locationData(:,3) = locationData(:,3) - lidarZoffset;
    locationData(:,3) = - locationData(:,3);

    quat1 = [odomData{i}.Pose.Pose.Orientation.W,...
        odomData{i}.Pose.Pose.Orientation.X,...
        odomData{i}.Pose.Pose.Orientation.Y,...
        odomData{i}.Pose.Pose.Orientation.Z];
    
    orientEuler = quat2eul(quat1);
%     orientEuler(2:3) = [0,0];
%     orientEuler(1) = orientEuler(1)*-1;
    orientEuler = orientEuler * -1;
%     rotm1 = quat2rotm(quat1);
    rotm1 = eul2rotm(orientEuler);
    
    locationDataR = locationData * rotm1;
%     curPos = curPos * rotm1;
    %     Get the location Data
    curPos = [odomData{i}.Pose.Pose.Position.X,...
            odomData{i}.Pose.Pose.Position.Y,...
            odomData{i}.Pose.Pose.Position.Z];
    
    if(exist('locationAllR','var'))
%         curPos = [odomData{i}.Pose.Pose.Position.X,...
%             odomData{i}.Pose.Pose.Position.Y, 0];
        checkPoints = 10;
        
        roiOffset = 0.03;
        roi = [curPos(1) - roiOffset,curPos(1) + roiOffset;...
            curPos(2) - roiOffset,curPos(2) + roiOffset;...
            curPos(3) + lidarZoffset - 2, curPos(3) + lidarZoffset + 2];
%         pcIndices = findPointsInROI(pcMergedTemp,roi);
%         [pcIndices,pcDists] = findNearestNeighbors(pcMergedTemp,curPos,checkPoints);        
%         pcIndicesClose = zeros(1,10,'double');
        locationROI = locationAllR(...
            locationAllR(:,1) >= roi(1,1) & locationAllR(:,1) <= roi(1,2) &...
            locationAllR(:,2) >= roi(2,1) & locationAllR(:,2) <= roi(2,2) &...
            locationAllR(:,3) >= roi(3,1) & locationAllR(:,3) <= roi(3,2),:);
%         if(~isempty(locationROI))
%             disp('GOTCHA');>= roi(2,1) & locationAllR(:,2) <= roi(2,2) &...
%         end
%         pcDistsXY = zeros(1,length(locationROI),'double');
%         for j = 1:length(pcIndices)
%             p1 = [curPos(1),curPos(2)];
%             p2 = [locationAllR(pcIndices(j),1),...
%                     locationAllR(pcIndices(j),2)];
%             pcDistsXY(j) = sum((p1-p2).^2).^0.5;
%             if(pcDistsXY(j) > 0.05)
%                 pcIndices(j) = 0;
%             end
%         end
%         pcIndicesClose = pcIndices(find(pcIndices));
        if(~isempty(locationROI))
            curPos(3) = mean(locationROI(:,3));
%             curPos(3) = curPos(3) - lidarZoffset;
            lidarHt = 1
        elseif(lidarHt == 1)
            disp('ERROR');
            curPos(3) = positionAll(i-1,3);
        end
    end

    locationDataPos = locationData + curPos;
    locationDataPosR = locationDataR + curPos;
%     pcGround{i} = select(pcObj{i},ringIndices);

%     Build the Pointcloud MATLAB class object
    intensityData = readField(lidarDataTemp{1},'intensity');
    if(useRings == 1)
        intensityData = intensityData(ringIndices);
    end
    pcGround{i} = pointCloud(locationDataPos, 'Intensity', intensityData);
    
    if(i == 1)
%         pcOut = pcmerge(pcGround{1}, pcGround{2}, 0.00001);
        initPos = curPos;
        locationAll = locationDataPos;
        locationAllR = locationDataPosR;
%         locationAll = [locationAll;locationDataPos];
%         locationAllR = [locationAllR;locationDataPosR];
    elseif(i == 2)
%         locationAll = locationDataPos;
%         locationAllR = locationDataPosR;
%         locationAll = [locationAll;locationDataPos];
            locationAll = [locationAll;locationDataPos];
            locationAllR = [locationAllR;locationDataPosR];
    elseif(i > 2)
%         pcOut = pcmerge(pcOut, pcGround{i}, 0.00001);
%         if(i == len)
            locationAll = [locationAll;locationDataPos];
            locationAllR = [locationAllR;locationDataPosR];
%         end
    end
    
%     pcMergedTemp = pointCloud(locationAllR);
%     pcMergedTemp = pcdownsample(pcMergedTemp, 'gridAverage', gridSize);
%     locationAllR = pcMergedTemp.Location;
    positionAll(i,:) = curPos;
end
colorDataG = locationAll;
colorDataG(:,1,:) = 0;
colorDataG(:,2,:) = 255;
colorDataG(:,3,:) = 0;
colorDataR = locationAll;
colorDataR(:,1,:) = 255;
colorDataR(:,2,:) = 0;
colorDataR(:,3,:) = 0;

pcMergedAll = pointCloud(locationAll,'Color',colorDataG);
% pcMergedAllR = pointCloud(locationAllR,'Color',colorDataR);
pcMergedAllR = pointCloud(locationAllR);

gridSize = 0.1;
% pcMergedAllR = pcdownsample(pcMergedAllR, 'gridAverage', gridSize);

figure,pcshow(pcMergedAllR);
hold on;
% pcshow(pcMergedAll);
% plot3(initPos(1),initPos(2),initPos(3),'go','MarkerSize',10,'MarkerFaceColor','g');
plot3(positionAll(:,1),positionAll(:,2),positionAll(:,3),'go','MarkerSize',5,'MarkerFaceColor','g');
plot3(curPos(1),curPos(2),curPos(3),'ro','MarkerSize',10,'MarkerFaceColor','r');
hold off;
figure,plot(timeOdom - timeLidar);
% figure,plot(timeOdom);
% figure,plot(seqLidar);
hold on;
legend;
% plot(seqOdom);
hold off;
plot(timeLidar);
figure,plot3(positionAll(:,1),positionAll(:,2),positionAll(:,3));
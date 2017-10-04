
[~,pcLen] = size(lidarData);
pcOld = pointCloud(readXYZ(lidarData(1)),'Intensity',readField(lidarData(1),'intensity'));
for i = 2:pcLen
    pcTemp = pointCloud(readXYZ(lidarData(i)),'Intensity',readField(lidarData(i),'intensity'));
    pcOut = pcmerge(pcTemp, pcOld, 0.0001);
    pcOld = pcOut;
end
pcshow(pcOut)
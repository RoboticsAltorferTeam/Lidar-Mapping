
[~,pcLen] = size(lidarData);
pcObj(1) = pointCloud(readXYZ(lidarData(1)),'Intensity',readField(lidarData(1),'intensity'));
for i = 2:pcLen
    pcObj(i) = pointCloud(readXYZ(lidarData(i)),'Intensity',readField(lidarData(i),'intensity'));
    pcOut = pcmerge(pcObj(i), pcObj(i-1), 0.0001);
%     pcOld = pcTemp;
end
pcshow(pcOut)
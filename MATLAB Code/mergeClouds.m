
[pcLen,~] = size(lidarData);
% pcLen = 252;
% pcOld = pointCloud(readXYZ(lidarData(1)),'Intensity',readField(lidarData(1),'intensity'));
% for i = 2:pcLen
%     pcTemp = pointCloud(readXYZ(lidarData(i)),'Intensity',readField(lidarData(i),'intensity'));
%     pcOut = pcmerge(pcTemp, pcOld, 0.0001);
%     pcOld = pcOut;
% end
pcCount = 1;

pcObj{pcCount} = pointCloud(readXYZ(lidarData{1}),'Intensity',readField(lidarData{1},'intensity'));
pcCount = pcCount + 1;
for i = 2:pcLen
    i
    pcObj{pcCount} = pointCloud(readXYZ(lidarData{i}),'Intensity',readField(lidarData{i},'intensity'));
    pcCount = pcCount + 1;
%     pcOut = pcmerge(pcObj{i}, pcObj{i-1}, 0.0001);
%     pcOld = pcOut;
end
% pcshow(pcOut)

if(~exist('lidarData','var'))
    if(~exist('bagFilename','var'))
        bagFilename = 'velo_1210_5.bag';
    end
    veloBag = rosbag(bagFilename);
    disp('File Imported');
    
    topicsTable = veloBag.AvailableTopics;
    numMessages = topicsTable{1,1};
    
    lidarDataTemp = readMessages(veloBag,1);
    pcObjTemp = pointCloud(readXYZ(lidarDataTemp{1}),'Intensity',...
        readField(lidarDataTemp{1},'intensity'));
    pcObj = cell(numMessages,1);
    pcObj{1} = pcObjTemp;
    
    for i = 2:numMessages
        i
%         hello
        lidarDataTemp = readMessages(veloBag,i);
        pcObj{i} = pointCloud(readXYZ(lidarDataTemp{1}),'Intensity',...
            readField(lidarDataTemp{1},'intensity'));
    end
else
    [pcLen,~] = size(lidarData);
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
end
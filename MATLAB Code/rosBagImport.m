clear

bagFilename = 'velo_1310_1.bag';
if(~exist(bagFilename,'file'))
    if(~exist('rosDevice1','var'))
        rosDevice = rosdevice('10.10.10.101','administrator','clearpath');
%         rosDevice1 = rosdevice('192.168.43.79','administrator','clearpath');
    end
    tic
    getFile(rosDevice,['~/', bagFilename]);
    toc
    disp('File Received');
end

veloBag = rosbag(bagFilename);
disp('File Imported');

topicsTable = veloBag.AvailableTopics;
numMessages = topicsTable{1,1};
if(numMessages <= 200)
    lidarData2 = readMessages(veloBag);
    disp('Data Extracted');
    lidarData = repmat(readMessages(veloBag,1),numMessages,1);
    for i = 1:ceil(numMessages/200)
        fromVal = (((i-1)*200) + 1);
        toVal = i*200;
        if(toVal > numMessages)
            toVal = numMessages;
        end
        lidarData(fromVal:toVal) = readMessages(veloBag,fromVal:toVal);
    end
end
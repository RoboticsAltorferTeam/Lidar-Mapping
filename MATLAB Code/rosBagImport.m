clear

bagFilename = 'velo_1710_5.6.bag';
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

rosBagAll = rosbag(bagFilename);
disp('File Imported');
% veloBag = select(rosBagAll,'Topic','/velodyne_points_2Hz');
% odomBag = select(rosBagAll,'Topic','odom_2Hz');
veloBag = select(rosBagAll,'Topic','/velodyne_points');
odomBag = select(rosBagAll,'Topic','/odometry/filtered');

odomData = readMessages(odomBag);
topicsTable = veloBag.AvailableTopics;
numMessages = topicsTable{1,1};
if(numMessages <= 200)
    lidarData = readMessages(veloBag);
    disp('Data Extracted');
%     lidarData = repmat(readMessages(veloBag,1),numMessages,1);
%     for i = 1:ceil(numMessages/200)
%         fromVal = (((i-1)*200) + 1);
%         toVal = i*200;
%         if(toVal > numMessages)
%             toVal = numMessages;
%         end
%         lidarData(fromVal:toVal) = readMessages(veloBag,fromVal:toVal);
%     end
end
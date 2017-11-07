clear

bagFilename = 'velo_0711_1.bag';
if(~exist(bagFilename,'file'))
    if(~exist('rosDevice1','var'))
        [~,current_ssid_out] = system('netsh wlan show interfaces | findstr /r "^....SSID"');
        [~,len1] = size(current_ssid_out);
        current_ssid_string = strip(current_ssid_out(30:len1));
        if(strcmp(current_ssid_string,'JackalRobotLab5G'))
            rosDevice1 = rosdevice('10.10.10.101','administrator','clearpath');
        elseif(strcmp(current_ssid_string,'RajRedmi'))
            rosDevice1 = rosdevice('192.168.43.79','administrator','clearpath');
        else
            rosDevice1 = rosdevice('192.168.1.11','administrator','clearpath');
        end
    end
    disp('File Receive Started');
    tic
    getFile(rosDevice1,['~/', bagFilename]);
    toc
    disp('File Received');
end

rosBagAll = rosbag(bagFilename);
disp('File Imported');
topicsTable2 = rosBagAll.AvailableTopics;
[numTopics,~] = size(topicsTable2);
for i = 1:numTopics
    if(topicsTable2.Row{i}(2:5) == 'odom')
        odomBag = select(rosBagAll,'Topic',topicsTable2.Row{i});
    elseif(topicsTable2.Row{i}(2:5) == 'velo')
        veloBag = select(rosBagAll,'Topic',topicsTable2.Row{i});
    end
end

% veloBag = select(rosBagAll,'Topic','/velodyne_points');
% odomBag = select(rosBagAll,'Topic','/odometry/filtered');
% odomBag = select(rosBagAll,'Topic','/jackal_velocity_controller/odom');

if(exist('odomBag','var'))
    odomData = readMessages(odomBag);
end
if(exist('veloBag','var'))
    topicsTable = veloBag.AvailableTopics;
    numMessages = topicsTable{1,1};
    if(numMessages <= 150)
        lidarData = readMessages(veloBag);
        disp('Data Extracted');
    end
end
clear

% velo_0212_2 X
% velo_0212_3 Y
% velo_0212_4 Y
% velo_0212_5 Y
% velo_0212_6 X

bagFilename = 'velo_0212_5.bag';
if(~exist(bagFilename,'file'))
    if(~exist('rosDevice1','var'))
        [~,eth_out] = system('netsh interface show interface | findstr /r "Ethernet"');
        if(strcmp(eth_out(74),'C'))
            rosDevice1 = rosdevice('192.168.1.11','administrator','clearpath');
        else
            [~,current_ssid_out] = system('netsh wlan show interfaces | findstr /r "^....SSID"');
            [~,len1] = size(current_ssid_out);
            current_ssid_string = strip(current_ssid_out(30:len1));
            if(strcmp(current_ssid_string,'JackalRobotLab5G'))
                rosDevice1 = rosdevice('10.10.10.101','administrator','clearpath');
            elseif(strcmp(current_ssid_string,'RajRedmi'))
                rosDevice1 = rosdevice('192.168.43.79','administrator','clearpath');
            end
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
    if(length(topicsTable2.Row{i}) >= 13)
        if(topicsTable2.Row{i}(2:13) == 'odometry/fil')
            odomFilBag = select(rosBagAll,'Topic',topicsTable2.Row{i});
            odomFilData = readMessages(odomFilBag);
        elseif(topicsTable2.Row{i}(2:13) == 'jackal_veloc')
    %     if(topicsTable2.Row{i}(2:13) == 'odometry/gps')
            odomBag = select(rosBagAll,'Topic',topicsTable2.Row{i});
            odomData = readMessages(odomBag);
        elseif(topicsTable2.Row{i}(2:5) == 'velo')
            veloBag = select(rosBagAll,'Topic',topicsTable2.Row{i});
            topicsTable = veloBag.AvailableTopics;
            numMessages = topicsTable{1,1};
            if(numMessages <= 150)
                lidarData = readMessages(veloBag);
                disp('Data Extracted');
            end
        end
    end
end

% veloBag = select(rosBagAll,'Topic','/velodyne_points');
% odomBag = select(rosBagAll,'Topic','/odometry/filtered');
% odomBag = select(rosBagAll,'Topic','/jackal_velocity_controller/odom');

% if(exist('odomBag','var'))
%     
% end
% if(exist('odomFilBag','var'))
%     
% end
% if(exist('veloBag','var'))
% 
% end
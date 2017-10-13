clear
% rosDevice = rosdevice('10.10.10.101','administrator','clearpath');
rosDevice = rosdevice('192.168.43.79','administrator','clearpath');
bagFilename = 'velo4.bag';
tic
getFile(rosDevice,['~/', bagFilename]);
toc
disp('File Received');
veloBag = rosbag(bagFilename);
disp('File Imported');
lidarData = readMessages(veloBag);
disp('Data Extracted');
toc
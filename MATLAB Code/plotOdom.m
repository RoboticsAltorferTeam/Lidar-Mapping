odomData = odomFilData;

[len2,~] = size(odomData);
positionAll = zeros(len2,3,'double');
orientQuat = zeros(len2,4,'double');
orientEuler = zeros(len2,3,'double'); %ZYX
time1 = zeros(len2,1,'double');
for i = 1:len2
    positionAll(i,:) = [odomData{i}.Pose.Pose.Position.X,...
        odomData{i}.Pose.Pose.Position.Y,...
        odomData{i}.Pose.Pose.Position.Z];
    
    orientQuat(i,:) = [odomData{i}.Pose.Pose.Orientation.W,...
        odomData{i}.Pose.Pose.Orientation.X,...
        odomData{i}.Pose.Pose.Orientation.Y,...
        odomData{i}.Pose.Pose.Orientation.Z];
    
    orientEuler(i,:) = quat2eul(orientQuat(i,:)); %ZYX
    
    time1(i) = odomData{i}.Header.Stamp.Sec + (odomData{i}.Header.Stamp.Nsec * 1e-9);
    if(i == 1)
        offset = time1(i);
%         time1(1) = 0;
    end
    time1(i) = time1(i) - offset;
end
figure;
% plot3(positionAll(:,1),positionAll(:,2),positionAll(:,3));
plot(time1,positionAll);
% plot(positionAll(:,1),positionAll(:,2));
% plot(orientQuat);
figure, plot(time1,orientEuler);
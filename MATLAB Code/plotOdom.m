[len2,~] = size(odomData);

positionAll = zeros(len2,3,'double');
orientQuat = zeros(len2,4,'double');
orientEuler = zeros(len2,3,'double'); %ZYX
for i = 1:len2
    positionAll(i,:) = [odomData{i}.Pose.Pose.Position.X,...
        odomData{i}.Pose.Pose.Position.Y,...
        odomData{i}.Pose.Pose.Position.Z];
    
    orientQuat(i,:) = [odomData{i}.Pose.Pose.Orientation.W,...
        odomData{i}.Pose.Pose.Orientation.X,...
        odomData{i}.Pose.Pose.Orientation.Y,...
        odomData{i}.Pose.Pose.Orientation.Z];
    
    orientEuler(i,:) = quat2eul(orientQuat(i,:)); %ZYX
end
% plot(positionAll);
% plot(orientQuat);
figure, plot(orientEuler);
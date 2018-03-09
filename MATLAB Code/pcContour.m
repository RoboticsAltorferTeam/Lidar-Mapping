pcLocationAllR = pcMergedAllR.Location;
pcLen1 = length(pcLocationAllR);
mid1 = ceil(pcLen1/2);

% xVals = zeros(mid1,2,'double');
% yVals = zeros(mid1,2,'double');
% zVals = zeros(mid1,2,'double');

% xVals = [pcLocationAllR(1:mid1,1),pcLocationAllR(mid1:pcLen1,1)];
% yVals = [pcLocationAllR(1:mid1,2),pcLocationAllR(mid1:pcLen1,2)];
% zVals = [pcLocationAllR(1:mid1,3),pcLocationAllR(mid1:pcLen1,3)];
gridSize = 10;

xVals = pcMergedAllR.XLimits(1):gridSize:pcMergedAllR.XLimits(2);
yVals = pcMergedAllR.YLimits(1):gridSize:pcMergedAllR.YLimits(2);

xLen = length(xVals)-1
yLen = length(yVals)-1

zVals = zeros(yLen,xLen,'double');

for i = 1:xLen
    i
    tic
    for j = 1:yLen
        roi = [xVals(i),xVals(i+1); yVals(j),yVals(j+1);...
            pcMergedAllR.ZLimits(1),pcMergedAllR.ZLimits(2)];
        indices = findPointsInROI(pcMergedAllR,roi);
%         pcROI = pcLocationAllR(...
%             pcLocationAllR(:,1) >= roi(1,1) & pcLocationAllR(:,1) < roi(1,2) &...
%             pcLocationAllR(:,2) >= roi(2,1) & pcLocationAllR(:,2) < roi(2,2),:);
        if(~isempty(indices))
%         if(~isempty(pcROI))
            pcROI = select(pcMergedAllR,indices);
            zVals(j,i) = pcROI.ZLimits(2); 
%             zVals(j,i) = max(pcROI(:,3));
        else
            zVals(j,i) = nan;
        end
    end
    toc
end

figure, contour(xVals(1:xLen),yVals(1:yLen),zVals);
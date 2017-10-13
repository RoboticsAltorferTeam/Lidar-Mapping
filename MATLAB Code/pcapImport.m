fileFilter = '*.pcap'; 
[File_name,Directory]=uigetfile(fileFilter,'Open a .pcap file');
Filename=[Directory File_name]; 
tic; fid=fopen(Filename); ttc=fread(fid,40); ttc=fread(fid,42); 
ttc=fread(fid,inf,'1206*uint8=>uint8',58);
%ttch=dec2hex(ttc);

% Determine how many data packets. 
Packet=size(ttc)/1206;
% Convert data to single precision. 
S1=single(ttc(2,:))*256+single(ttc(1,:)); 
S2=single(ttc(102,:))*256+single(ttc(101,:)); 
S3=single(ttc(202,:))*256+single(ttc(201,:)); 
S4=single(ttc(302,:))*256+single(ttc(301,:)); 
for i=0:10000 % Packets loop 
    status(i+1)=(ttc(1205+i*1206)); value(i+1)=(ttc(1206+i*1206));
end
a=[85 78 73 84 35]
fclose(fid);
toc; 
Ind=strfind(value,a); 

% Loop through 64 lasers. 
for i=1:16 
    temp=single(value(Ind(1)+64*(i-1)+16:Ind(1)+64*(i-1)+16+7));
    temp1=single(value(Ind(1)+64*(i-1)+32:Ind(1)+64*(i-1)+32+7)); 
    temp2=single(value(Ind(1)+64*(i-1)+48:Ind(1)+64*(i-1)+48+7)); 
    temp3=single(value(Ind(1)+64*(i-1)+64:Ind(1)+64*(i-1)+64+7)); 
    LaserId(i)=temp(1); 
    
% Add high and low bytes of Vertical Correction Factor together and check if positive or negative correction factor. 
VerticalCorr(i)=temp(3)*256+temp(2);
if VerticalCorr(i)>32768 
    VerticalCorr(i)=VerticalCorr(i)-65536;
end 

% Scale Vertical Correction Factor by Dividing by 100. 
VerticalCorr(i)=VerticalCorr(i)/100;

% Add high and low bytes of Rotational Correction Factor together and check if positive or negative correction factor. 
RotationalCorr(i)=temp(5)*256+temp(4);
if RotationalCorr(i)>32768
    RotationalCorr(i)=RotationalCorr(i)-65536;
end 

% Scale Rotational Correction Factor by Dividing by 100.
RotationalCorr(i)=RotationalCorr(i)/100;

% Add high and low bytes of remaining 2 Byte Correction Factors together and check if positive or negative correction factor, if necessary. Scale dimensions in mm to cm by Dividing by 10. Scale Focal Slope by Dividing by 10. 
DistanceCorr(i)=(temp(7)*256+temp(6))/10;
DistanceCorrX(i)=(temp1(2)*256+temp1(1))/10;
DistanceCorrY(i)=(temp1(4)*256+temp1(3))/10; 
VerticalOffset(i)=(temp1(6)*256+temp1(5))/10; 
HorizonOffset(i)=(temp2(1)*256+temp1(7)); 

if HorizonOffset(i)>32768 
    HorizonOffset(i)=HorizonOffset(i)-65536; 
end
HorizonOffset(i)=HorizonOffset(i)/10;
FocalDist(i)=temp2(3)*256+temp2(2); 

if  FocalDist(i)>32768 
    FocalDist(i)=FocalDist(i)-65536; 
end
FocalDist(i)=FocalDist(i)/10;
FocalSlope(i)=temp2(5)*256+temp2(4); 

if  FocalSlope(i)>32768 
    FocalSlope(i)=FocalSlope(i)-65536; 
end
FocalSlope(i)=FocalSlope(i)/10;

% Maximum and Minimum Intensity only 1 Byte each.
MinIntensity(i)=temp2(6); 
MaxIntensity(i)=temp2(7); 
end

% Done with correction factors.
% Get Unit Parameter Data 
% s=Ind(1) char(status(s-80:s+6)) value(s-80:s+6) 
% Version=dec2hex(value(s-1)) 
% Temperature = value(s-2) 
% GPS=value(s-3) 
% speed=single(value(s-48))+single(value(s-47))*256 
% Fov^start=single(value(s-46))+single(value(s-45))*256
% Fov^end=single(value(s-44))+single(value(s-43))*256 
% warning=value(s-13)
% power=value(s-12) 
% Humidity=value(s-58) 
% Done with Unit Parameters.
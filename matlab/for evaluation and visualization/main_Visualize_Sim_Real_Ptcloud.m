clc; clear all; close all;


%% Load lidar data

% Sim Data
% fileDir = 'D:/03_Datasets\OneDrive_2020-12-08 (1)/'; 
fileDir = './input data/teamviewer3/';
fileName = '16Channel_LidarCS_2887_Frames.csv';
simLidarData = readmatrix([fileDir, fileName], 'Delimiter', ',', 'NumHeaderLines', 1);

simDistance = simLidarData(:, 3);
simAzimuth = simLidarData(:, 4);
simVertAngle = -simLidarData(:, 5);

x = simDistance.* sind(simAzimuth) .* cosd(simVertAngle);
y = simDistance.* cosd(simAzimuth) .* cosd(simVertAngle);
z = simDistance.* sind(simVertAngle);

simPoint = horzcat(x, y, z);
simIntensity = simLidarData(:, 6);
clear x y z simLidarData simDistance simAzimuth simVertAngle

% Real Data
lidarFileName = './input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';
veloReader = velodyneFileReader(lidarFileName, 'VLP16');


%% Play both lidar point cloud

xlimits = [-60 60];
ylimits = [-60 60];
zlimits = [-20 20];

simPlayer = pcplayer(xlimits,ylimits,zlimits);
realPlayer = pcplayer(xlimits,ylimits,zlimits);
simPlayer.Axes.CLim = [0, 100];
realPlayer.Axes.CLim = [0, 100];

xlabel(simPlayer.Axes,'X (m)'); ylabel(simPlayer.Axes,'Y (m)'); zlabel(simPlayer.Axes,'Z (m)');
xlabel(realPlayer.Axes,'X (m)'); ylabel(realPlayer.Axes,'Y (m)'); zlabel(realPlayer.Axes,'Z (m)');

i = 0;
pointsPerFrame = 3200;
while(simPlayer.isOpen() && i < size(simPoint, 1))
    
    if (i + pointsPerFrame < size(simPoint, 1))
        simPtCloud = pointCloud(simPoint(i+1:i+pointsPerFrame, :), 'Intensity', simIntensity(i+1:i+pointsPerFrame));
    else
        simPtCloud = pointCloud(simPoint(i+1:end, :), 'Intensity', simIntensity(i+1:end));
    end   
    
    realPtCloud = readFrame(veloReader);
    
    pause(0.065);
    
    view(simPlayer,simPtCloud.Location, simPtCloud.Intensity); 
    view(realPlayer,realPtCloud.Location, realPtCloud.Intensity); 
    
%     % Check minimum distance of each point cloud
%     simLoc = simPtCloud.Location;
%     realLoc = reshape(realPtCloud.Location, [], 3); 
%     simDist = sqrt(simLoc(:, 1).^2 + simLoc(:, 2).^2 + simLoc(:, 3).^2);
%     simDist = simDist(simDist~=0);
%     realDist = sqrt(realLoc(:, 1).^2 + realLoc(:, 2).^2 + realLoc(:, 3).^2);
%     realDist = realDist(~isnan(realDist));    
%     min(simDist)
%     min(realDist)   
    
    i = i+pointsPerFrame;    
    
end


%% Load real lidar data

lidarFileName = './input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';

veloReader = velodyneFileReader(lidarFileName, 'VLP16');

xlimits = [-60 60];
ylimits = [-60 60];
zlimits = [-20 20];

simPlayer = pcplayer(xlimits,ylimits,zlimits);

xlabel(simPlayer.Axes,'X (m)');
ylabel(simPlayer.Axes,'Y (m)');
zlabel(simPlayer.Axes,'Z (m)');

veloReader.CurrentTime = veloReader.StartTime + seconds(0.3); 

while(hasFrame(veloReader) && simPlayer.isOpen() )
    simPtCloud = readFrame(veloReader);
    view(simPlayer,simPtCloud.Location,simPtCloud.Intensity);
    pause(0.1);
end






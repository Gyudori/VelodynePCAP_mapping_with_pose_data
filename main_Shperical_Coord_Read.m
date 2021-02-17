clc; clear all; close all;


%% Load lidar data

simLidarData = readmatrix('./input data/16Channel_LidarCS_31_Frames.csv', 'NumHeaderLines', 1, 'Delimiter', ',');


simDistance = simLidarData(:, 3);
simAzimuth = simLidarData(:, 4);
simVertAngle = -simLidarData(:, 5);
simIntensity = simLidarData(:, 6);

x = simDistance.* sind(simAzimuth) .* cosd(simVertAngle);
y = simDistance.* cosd(simAzimuth) .* cosd(simVertAngle);
z = simDistance.* sind(simVertAngle);

point = horzcat(x, y, z);

lidarFileName = './input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';

veloReader = velodyneFileReader(lidarFileName, 'VLP16');

%% Play only sim pt

xlimits = [-60 60];
ylimits = [-60 60];
zlimits = [-20 20];

player1 = pcplayer(xlimits,ylimits,zlimits);

xlabel(player1.Axes,'X (m)');
ylabel(player1.Axes,'Y (m)');
zlabel(player1.Axes,'Z (m)');

i = 0;
pointsPerFrame = 28800;
while(player1.isOpen() && i < size(point, 1))
    
    if (i + pointsPerFrame < size(point, 1))
        simPtCloudObj = pointCloud(point(i+1:i+pointsPerFrame, :), 'Intensity', simIntensity(i+1:i+pointsPerFrame));
    else
        simPtCloudObj = pointCloud(point(i+1:end, :), 'Intensity', simIntensity(i+1:end));
    end   
    
    view(player1,simPtCloudObj.Location,simPtCloudObj.Intensity);    
    pause(0.1);
    
    i = i+pointsPerFrame;
end


%% Play both lidar point cloud

xlimits = [-60 60];
ylimits = [-60 60];
zlimits = [-20 20];

player1 = pcplayer(xlimits,ylimits,zlimits);
player2 = pcplayer(xlimits,ylimits,zlimits);

xlabel(player1.Axes,'X (m)');
ylabel(player1.Axes,'Y (m)');
zlabel(player1.Axes,'Z (m)');
xlabel(player2.Axes,'X (m)');
ylabel(player2.Axes,'Y (m)');
zlabel(player2.Axes,'Z (m)');

i = 0;
pointsPerFrame = 1600;
while(player1.isOpen() && i < size(point, 1))
    
    if (i + pointsPerFrame < size(point, 1))
        simPtCloudObj = pointCloud(point(i+1:i+pointsPerFrame, :), 'Intensity', simIntensity(i+1:i+pointsPerFrame));
    else
        simPtCloudObj = pointCloud(point(i+1:end, :), 'Intensity', simIntensity(i+1:end));
    end   
    
    realPtCloudObj = readFrame(veloReader);
    
    view(player1,simPtCloudObj.Location,simPtCloudObj.Intensity);
    view(player2,realPtCloudObj.Location,realPtCloudObj.Intensity);
    pause(0.1);
    
    i = i+pointsPerFrame;
end


%% Load real lidar data

lidarFileName = './input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';

veloReader = velodyneFileReader(lidarFileName, 'VLP16');

xlimits = [-60 60];
ylimits = [-60 60];
zlimits = [-20 20];

player1 = pcplayer(xlimits,ylimits,zlimits);

xlabel(player1.Axes,'X (m)');
ylabel(player1.Axes,'Y (m)');
zlabel(player1.Axes,'Z (m)');

veloReader.CurrentTime = veloReader.StartTime + seconds(0.3); 

while(hasFrame(veloReader) && player1.isOpen() )
    simPtCloudObj = readFrame(veloReader);
    view(player1,simPtCloudObj.Location,simPtCloudObj.Intensity);
    pause(0.1);
end






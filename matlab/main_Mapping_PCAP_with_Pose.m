%%%%%%%%%%%%%%%%%%%%%%
% LIDAR mapping code
%%%%%%%%%%%%%%%%%%%%%%

clear all
close all
clc

% Generate repeatable random variables
rng(1);


%% Choose region of data
% region = "Sangam";
region = "UOS";

if region == "Sangam"
    lidarFileName = '../input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';
    poseFileName = "../input data/Sangam/Sangam_Mission 1.txt";
    
elseif region == "UOS"
    lidarFileName = '../input data/UOS/2020-10-28-14-17-42_Velodyne-VLP-16-Data.pcap';
    poseFileName = "../input data/UOS/UOS_Mission 1.txt";
end

%% Read velodyne pcap file 

% Load lidar data
veloReader = velodyneFileReader(lidarFileName, 'VLP16');

% Total number of frames
% numberOfFrames = veloReader.NumberOfFrames;
numberOfFrames = 100;

% Sampling rate
frameInterval = 1;
lidarDataSize = ceil(numberOfFrames/frameInterval);

% Create empty arrays
ptClouds = pointCloud.empty(0, lidarDataSize);
lidarTimestamps = duration(nan(lidarDataSize, 3));

% Save sampled data
ptCloudIndex = 0;
for i = 1:frameInterval:numberOfFrames
    ptCloudIndex = ptCloudIndex+1;
    ptClouds(ptCloudIndex) = readFrame(veloReader, i);
    lidarTimestamps(ptCloudIndex) = veloReader.Timestamps(i);
end

% Save lidarData with time offset
timeOffsetSecond = 18; % leap seconds (윤초)
timeOffset = duration(0, 0, timeOffsetSecond);
lidarTimestamps = lidarTimestamps + timeOffset;

lidarTimestamps.Format = 'hh:mm:ss.SSSS';
lidarPointClouds = timetable(lidarTimestamps, ptClouds', 'VariableNames', {'ptClouds'});

% Extract translation offset


%% Visualization sample lidar point cloud  

figure;
pcshow(lidarPointClouds.ptClouds(1)); 
vis_coord_system ([0 0 0]', eye(3, 3), 10, '');
title('Point Cloud and Lidar frame axis');

% Preprocess
for i = 1:numberOfFrames
    lidarPointClouds.ptClouds(i) = SelectPointFartherThan(lidarPointClouds.ptClouds(i), 1.5);
end

ptTemp = pointCloud(excludedLoc, 'Intensity', excludedIntensity);
figure; hold on;
pcshow(excludedLoc, [1 1 1], 'MarkerSize', 100 );

pcshow(lidarPointClouds.ptClouds(1), 'MarkerSize', 1);


%% Pose data read
% Pose data format
% [1TIME, 2DISTANCE, 3EASTING, 4NORTHING, 5ELLIPSOID HEIGHT, 6LATITUDE, 7LONGITUDE, 8ELLIPSOID HEIGHT, 
% 9ROLL, 10PITCH, 11HEADING, 12EAST VELOCITY, 13NORTH VELOCITY, 14UP VELOCITY, 
% 15EAST SD, 16NORTH SD, 17HEIGHT SD, 18ROLL SD, 19PITCH SD, 20HEADING SD]

% Load pose data
poseData = importdata(poseFileName,' ',0);
poseDataSize = size(poseData, 1);

% Save imuData with time stamp
poseTimestamps = seconds(mod(poseData(:, 1), 3600));
poseTimestamps.Format = 'hh:mm:ss.SSSS';
Poses = timetable(poseTimestamps, poseData(:, 3), poseData(:, 4), poseData(:, 5),...
    poseData(:, 9)*pi/180, poseData(:, 10)*pi/180, poseData(:, 11)*pi/180, 'VariableNames',...
    {'EASTING', 'NORTHING', 'HEIGHT', 'ROLL', 'PITCH', 'HEADING'});


% Compute Position and Rotation
Position = [Poses.EASTING, Poses.NORTHING, Poses.HEIGHT];
RotationMat = zeros(poseDataSize, 3, 3);
for i = 1:poseDataSize
    ra = [Poses.HEADING(i), Poses.PITCH(i), Poses.ROLL(i)];    
    RotationMat(i, :, :) = A2R_YPR ( ra );
end

%% Save rotation and position 

PosesRT = timetable(poseTimestamps, RotationMat, Position, 'VariableNames', {'Rotation', 'Position'});


%% Time synchronization

indices = zeros(lidarDataSize, 1);

for i = 1:lidarDataSize
    
    if(i == 1)
        n = 1;
    end    
    
    for j = n:poseDataSize        
        
        if(lidarPointClouds.lidarTimestamps(i) > PosesRT.poseTimestamps(j))            
            continue;   
        
        % pose의 시간이 조금 더 이후일 때의 pose 채택
        elseif(lidarPointClouds.lidarTimestamps(i) <= PosesRT.poseTimestamps(j))            
            indices(i) = j;
            n = j;
            break;
        end            
    end
end

PoseRTSync = PosesRT(indices, :);

%% Plotting - odometry and poses before axis rotation

[S L] = bounds(indices);
poseIndex = [S L];
PlotOdometryPose(PosesRT.Position, PosesRT.Rotation, eye(3), poseIndex)
title('odometry and poses before axis rotation');


%% Transform lidar point cloud with Pose data

ptCloudDownsampleTrans = pointCloud.empty(0, lidarDataSize);
for i = 1:lidarDataSize
    
    % Downsampling 
%     ptCloudDownsample = pcdownsample(lidarPointClouds(i, :).ptClouds,'random',0.1);
    ptCloudDownsample = lidarPotintClouds(i, :).ptClouds;
    points = reshape(ptCloudDownsample.Location, [], 3)';
    
    % Translation offset - set first pose as frame origin
    intialPosition = repmat(PoseRTSync(1, :).Position', 1, size(points, 2)); 

    % rotation and position
    axisRot = [1 0 0; 0 0 -1; 0 1 0];
    rotation = axisRot' * reshape(PoseRTSync(i, :).Rotation, 3, 3);
    posititon = repmat(PoseRTSync(i, :).Position', 1, size(points, 2)); 

    % Transformation
    pointsTrans =   double(rotation' * points) + posititon - intialPosition;  

    % Intensity for plotting
    intensity = reshape(ptCloudDownsample.Intensity, 1, []);
    ptCloudDownsampleTrans(i) = pointCloud(pointsTrans', 'Intensity', intensity');      
end


%% Plotting - point cloud map

% Merge point cloud for plotting
concatPtCloud = pccat(ptCloudDownsampleTrans);

% Plotting point cloud
figure; pcshow(concatPtCloud);

% % Plotting poses on the point cloud
% figure
% for i = 1:10:lidarDataSize
%     hold on     
%     vis_coord_system (PoseRTSync(i, :).Position', reshape(PoseRTSync(i, :).Rotation, 3, 3)', 10, num2str(i));  
% end
% grid on; axis equal; xlim auto; ylim auto;
% ax = gca;
% ax.Clipping = 'off';


%% Merging point cloud

if region == "Sangam"
    ouputDir = "../output data/Sangam/";
    outputFileName = strcat(ouputDir, "object3d_",  num2str(timeOffsetSecond),  ".ply");
    pcwrite(concatPtCloud,outputFileName,'Encoding','ascii');
elseif region == "UOS"
    ouputDir = "../output data/UOS/";
    outputFileName = strcat(ouputDir, "object3d_",  num2str(timeOffsetSecond),  ".ply");
    pcwrite(concatPtCloud,outputFileName,'Encoding','ascii');
end














%% GPS second to date example
% gpsWeekStartDay = [2020,10,25];
% gpsWeekStart = datetime(gpsWeekStartDay,'TimeZone','UTC')
% timestamp = gpsWeekStart + seconds(gpsSecondsOfWeek)
% localTimestamp = datetime(timestamp,'TimeZone','Asia/Seoul')





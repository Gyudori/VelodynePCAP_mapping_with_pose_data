clear all
close all
clc

% Generate repeatable random variables
rng(1);


%% Choose region of data
region = "Sangam";
% region = "UOS";

if region == "Sangam"
    lidarFileName = './input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';
    poseFileName = "./input data/Sangam/Sangam_Mission 1.txt";
    
elseif region == "UOS"
    lidarFileName = './input data/UOS/2020-10-28-14-17-42_Velodyne-VLP-16-Data.pcap';
    poseFileName = "./input data/UOS/UOS_Mission 1.txt";
end

%% Read velodyne pcap file 

veloReader = velodyneFileReader(lidarFileName, 'VLP16');

% numberOfFrames = veloReader.NumberOfFrames;
numberOfFrames = 500;

frameSamplingInterval = 1;
lidarDataSize = ceil(numberOfFrames/frameSamplingInterval);

lidarPtClouds = pointCloud.empty(0, lidarDataSize);
lidarTimestamps = duration(nan(lidarDataSize, 3));

% Save sampled data
arrayIndex = 0;
for frameIndex = 1:frameSamplingInterval:numberOfFrames
    arrayIndex = arrayIndex+1;
    lidarPtClouds(arrayIndex) = readFrame(veloReader, frameIndex);
    lidarTimestamps(arrayIndex) = veloReader.Timestamps(frameIndex);
end

% Save lidarData with time stamp
leapSecondsTimeOffset = 18; % leap seconds (윤초)
timeOffset = duration(0, 0, leapSecondsTimeOffset);
lidarTimestamps = lidarTimestamps + timeOffset;

lidarTimestamps.Format = 'hh:mm:ss.SSSS';
lidarPointClouds = timetable(lidarTimestamps, lidarPtClouds', 'VariableNames', {'ptClouds'});

%% Visualization sample lidar point cloud  

% figure;
% pcshow(lidarPointClouds.ptClouds(1)); 
% vis_coord_system ([0 0 0]', eye(3, 3), 10, '');
% title('Point Cloud and Lidar frame axis');


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
posesNamed = timetable(poseTimestamps, poseData(:, 3), poseData(:, 4), poseData(:, 5),...
    poseData(:, 9)*pi/180, poseData(:, 10)*pi/180, poseData(:, 11)*pi/180, 'VariableNames',...
    {'easting', 'northing', 'height', 'roll', 'pitch', 'heading'});


% Compute Position and Rotation
position = [posesNamed.easting, posesNamed.northing, posesNamed.height];
rotationMat = zeros(poseDataSize, 3, 3);
for i = 1:poseDataSize
    rotationAngle = [posesNamed.heading(i), posesNamed.pitch(i), posesNamed.roll(i)];    
    rotationMat(i, :, :) = A2R_YPR ( rotationAngle );
end

%% Save rotation and position 

posesRotPosAng = timetable(poseTimestamps, rotationMat, position,...
    poseData(:, 9), poseData(:, 10), poseData(:, 11),...
    'VariableNames', {'Rotation', 'Position', 'roll', 'pitch', 'heading'});


%% Time synchronization

syncIndices = zeros(lidarDataSize, 1);

n = 1;
for i = 1:lidarDataSize
    for j = n:poseDataSize
        if(lidarPointClouds.lidarTimestamps(i) > posesRotPosAng.poseTimestamps(j))            
            continue;
        % pose의 시간이 조금 더 이후일 때의 pose 채택
        elseif(lidarPointClouds.lidarTimestamps(i) <= posesRotPosAng.poseTimestamps(j))            
            syncIndices(i) = j;
            n = j;
            break;
        end            
    end
end

PoseSync = posesRotPosAng(syncIndices, :);


%% Pose sampling

% Extract Lidar scanned data
[poseIdxMin, poseIdxMax] = bounds(syncIndices);
lidarHz = 10;
imuHz = 200;
lastAddingMeasurement = imuHz/lidarHz - 1;
PoseSyncRangeSegment = posesRotPosAng(poseIdxMin:poseIdxMax + lastAddingMeasurement, :);

simulatorHz = 50;
imuSamplingInterval = imuHz / simulatorHz;

poseSync50Hz = DownSample(PoseSyncRangeSegment, imuSamplingInterval);
poseExport = timetable2table(poseSync50Hz);

writetable(poseExport(:, 3:6), 'D:\01_LidarSimIntegrationBackup_1\merged_adSim_UnityPlugin_v0.4\Assets/PosEul_500.csv',...
    'WriteVariableNames', false);









%% Plotting

PlotOdometryPose(MatlabPositions, MatlabRoations, eye(3), [1, size(UnityPositions, 1)])
title('Matlab pose');

[xMin, xMax] = bounds(UnityPositions(:, 1));
[yMin, yMax] = bounds(UnityPositions(:, 2));
[zMin, zMax] = bounds(UnityPositions(:, 3));

PlotOdometryPose(UnityPositions, UnityRotations, eye(3), [1, size(UnityPositions, 1)])
hold on; vis_coord_system((UnityPositions(1, :)' + [100; 0; -100]), eye(3), 50, ''); 
title('Unity pose');
xlim([xMin, xMax]); ylim([yMin - 10, yMax + 10]); zlim([zMin, zMax]);


%% Pose to quaternion

% Rotation to quaternion
outputIndex = 0;
UnityQuaternion = zeros(outputSize, 4);
for i = 1:PoseInterval:size(PoseSyncRangeSegment, 1)
    outputIndex = outputIndex+1;
    UnityQuaternion(outputIndex, :) = rotm2quat(reshape(UnityRotations(outputIndex, :, :), 3, 3)');
    reshape(UnityRotations(outputIndex, :, :), 3, 3)'
    rotm2quat(reshape(UnityRotations(outputIndex, :, :), 3, 3)')
    
    % MATLAB(w,x,y,z) -> Unity(x,y,z,w)
    temp = UnityQuaternion(outputIndex, 1);
    UnityQuaternion(outputIndex, 1:3) = UnityQuaternion(outputIndex, 2:4);
    UnityQuaternion(outputIndex, 4) = temp;
end

%% Quaternion 



result = horzcat(UnityPositions, UnityQuaternion);

writematrix(result, './output data/UnityPosQuat.csv');



quat2rotm(quat)





    



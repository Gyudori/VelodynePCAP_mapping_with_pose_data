
%% Visualize pose data in WebMap

clear all
close all
clc

% Generate repeatable random variables
rng(1);


%% Choose region of data
region = "Sangam";
% region = "UOS";

if region == "Sangam"
    lidarFileName = '../../input data/Sangam/2020-11-05-10-35-44_Velodyne-VLP-16-Data.pcap';
    poseFileName = "../../input data/Sangam/Sangam_Mission 1.txt";
    
elseif region == "UOS"
    lidarFileName = '../../input data/UOS/2020-10-28-14-17-42_Velodyne-VLP-16-Data.pcap';
    poseFileName = "../../input data/UOS/UOS_Mission 1.txt";
end

%% Read velodyne pcap file 

% Load lidar data
veloReader = velodyneFileReader(lidarFileName, 'VLP16');

% Total number of frames
numberOfFrames = veloReader.NumberOfFrames;
% numberOfFrames = 500;

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

% Save lidarData with time stamp
timeOffsetSecond = 18; % leap seconds (윤초)
timeOffset = duration(0, 0, timeOffsetSecond);
lidarTimestamps = lidarTimestamps + timeOffset;

lidarTimestamps.Format = 'hh:mm:ss.SSSS';
lidarPointClouds = timetable(lidarTimestamps, ptClouds', 'VariableNames', {'ptClouds'});


%% Pose data read

% Load pose data
poseData = importdata(poseFileName,' ',0);
poseDataSize = size(poseData, 1);

% Save imuData with time stamp
poseTimestamps = seconds(mod(poseData(:, 1), 3600));
poseTimestamps.Format = 'hh:mm:ss.SSSS';
Poses = timetable(poseTimestamps, poseData(:, 6), poseData(:, 7), poseData(:, 8),...
    poseData(:, 9)*pi/180, poseData(:, 10)*pi/180, poseData(:, 11)*pi/180, 'VariableNames',...
    {'LAT', 'LON', 'HEIGHT', 'ROLL', 'PITCH', 'HEADING'});


% Compute Position and Rotation
Position = [Poses.LAT, Poses.LON, Poses.HEIGHT];
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



%% Plotting pose in Webmap

startingPoint.LAT = PoseRTSync.Position(1, 1);
startingPoint.LON = PoseRTSync.Position(1, 2);

webmap('World Imagery')
wmmarker(startingPoint.LAT, startingPoint.LON)

wmline(PoseRTSync.Position(:, 1), PoseRTSync.Position(:, 2), 'Color', 'r')

% show the track in a TM coordinate system
e = referenceEllipsoid('wgs84');
ax = axesm('MapProjection', 'tranmerc', 'Geoid', e, 'MapLatLimit', [37.5 37.6], 'MapLonLimit', [127.0 127.1], ...
    'Origin', [38 127], 'FalseEasting', 200000, 'FalseNorthing', 600000, ...
    'MeridianLabel', 'on', 'ParallelLabel', 'on', ...
    'PLineLocation', [37.5:0.02:37.6], 'MLineLocation', [127.0:0.02:127.1], ...
    'PLabelLocation', [37.5:0.02:37.6], 'MLabelLocation', [127.0:0.02:127.1], ...
    'PLabelRound', -2, 'MLabelRound', -2, ...
    'Frame', 'on', 'Grid', 'on');
geoshow(trk.Latitude, trk.Longitude);

% convert the lat. and long into x, y in TM
mstruct = getm(ax);
[x,y] = mfwdtran(mstruct, trk.Latitude, trk.Longitude);
pos_tm = [x', y']

pos_tm(1:10, :)

% Compute the mean and standard deviation
mean(pos_tm)
std(pos_tm)
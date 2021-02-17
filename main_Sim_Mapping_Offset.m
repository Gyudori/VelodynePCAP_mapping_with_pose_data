clc; clear all; close all;

fileName = '16Channel_Mapping_Result_2887_Frames.csv';
simLidarData = readmatrix(['./input data/teamviewer3/', fileName], 'NumHeaderLines', 1, 'Delimiter', ',');

x = simLidarData(:, 3);
y = simLidarData(:, 4);
z = simLidarData(:, 5);
i = simLidarData(:, 6);

xOffset = 313592.8473;
yOffset = 36;
zOffset = 4161038.8462;

xExport = x + xOffset;
yExport = z + zOffset;
zExport = y + yOffset;

horzcat

writematrix(horzcat(xExport, yExport, zExport, i), ['./output data/', fileName]);

clc
clear all
close all

% GCP visualization


%% Data geometry visualization

% Data input
% northing, easting, height
A =[553810.321  	204978.546 	62.271;
553815.406 	204975.597  	62.333;
553813.901 	204984.565 	62.022;
553818.961 	204981.599 	62.027;
553793.390	204986.327	73.269;
553793.359	204983.441	73.268;
553793.366	204986.311	70.582;
553793.375	204983.431	70.541;];

% visualization
figure
for i = 1:8    
    hold on
    [A(i, 2), A(i, 1), A(i, 3)]
    plot3(A(i, 2), A(i, 1), A(i, 3), '.', 'MarkerSize', 10);
end
grid on
axis equal


%% Visualization in a map

% Data input
% lat, lon
B = [37.58383535	127.0563653;
37.58388119	127.0563319;
37.58386758	127.0564334;
37.58391319	127.0563998;
37.58368277	127.0564532;
37.5836825	127.0564206;
37.58368255	127.0564531;
37.58368265	127.0564205];

webmap('World Imagery')
for i = 1:8
    wmmarker(B(i, 1), B(i, 2));
end





% 3D rotation matrix
% Impyeong Lee
% University of Seoul

% Gyuseok Lee
% 2020. 11. 24- [yaw(a), pitch(b), roll(c)] -> ra (rotation angle)
% Rotation matrix for APX-18 (Heading, pitch, roll)

function R = A2R_YPR ( ra )

% yaw, pitch, roll에 해당하는 축은 정의되는 좌표계에 의존
a = ra(1); % yaw - z축
c = ra(2); % pitch - x축
b = ra(3); % roll - y축 

Rx = [1 0 0; 0 cos(c) -sin(c); 0 sin(c) cos(c)];
Ry = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
Rz = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];

% 순서 또한 정의되는 좌표계에 의존
R = Ry * Rx * Rz; % yaw -> pitch -> roll 순서로 회전

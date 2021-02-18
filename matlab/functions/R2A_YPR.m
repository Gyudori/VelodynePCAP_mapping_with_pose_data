% 3D rotation matrix
% Impyeong Lee
% University of Seoul

% Gyuseok Lee
% 2020. 04. 04 : Rotation matrix -> Rotation angle
% R : Rz(yaw)*Ry(pitch)*Rx(roll)
% ra = [yaw(a), pitch(b), roll(c)]
% Reference : http://planning.cs.uiuc.edu/node102.html

function ra = R2A_YPR ( R )

sin_b = -R(3,1);
cos_b = zeros(2,1);
cos_b(1) = sqrt(1-sin_b^2);
cos_b(2) = -sqrt(1-sin_b^2);

c = zeros(2,1);
a = zeros(2,1);
c(1) = atan2(R(3,2), R(3,3));
a(1) = atan2(R(2,1), R(1,1));
c(2) = atan2(-R(3,2), -R(3,3));
a(2) = atan2(-R(2,1), -R(1,1));

err = zeros(2,1);
for n = 1:2
   err(n) = abs(cos(a(n))*sin_b*sin(c(n)) - sin(a(n))*cos(c(n)) - R(1,2));
   err(n) = err(n) + abs(cos(a(n))*sin_b*cos(c(n)) + sin(a(n))*sin(c(n)) - R(1,3));
   err(n) = err(n) + abs(sin(a(n))*sin_b*sin(c(n)) + cos(a(n))*cos(c(n)) - R(2,2));
   err(n) = err(n) + abs(sin(a(n))*sin_b*cos(c(n)) - cos(a(n))*sin(c(n)) - R(2,3));
end
[val,idx] = min(err);
if val > 1e-6,
    fprintf('Error!!');
end

ra(:,1) = a;
ra(:,2) = atan2(sin_b, cos_b);
ra(:,3) = c;


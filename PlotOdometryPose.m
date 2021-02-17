function PlotOdometryPose(Positions, Rotations, axisRot, poseIndex)

% Skipping parameters
odometryEnd = poseIndex(2);
poseInterval = round((poseIndex(2) - poseIndex(1)) / 100);
axisLength = 30;

% Plot odometry
figure; hold on;
plot3(Positions(1:odometryEnd, 1), Positions(1:odometryEnd, 2), Positions(1:odometryEnd, 3), '.');
grid on; axis equal; xlim auto; ylim auto;
ax = gca;
ax.Clipping = 'off';

% % plot reference frame axis
% originOffset = [100, 100, 0]';
% vis_coord_system(Position(1, :)' - originOffset, eye(3), axisLength, '');

% Plotting poses
for i = poseIndex(1):poseInterval:poseIndex(2)  
    vis_coord_system (Positions(i, :)', reshape(Rotations(i, :, :), 3, 3)' * axisRot', axisLength, num2str(i));
end

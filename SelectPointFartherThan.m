function ptCloud = SelectPointFartherThan(ptCloud, minRange)

% Preprocessing 
pointLoc = ptCloud.Location;
pointInt = ptCloud.Intensity;

excludedLoc = [];
excludedIntensity = [];

for i = 1:size(pointLoc, 1)
    for j = 1:size(pointLoc, 2)
        if ~isnan(pointLoc(i, j, :))
            dist = norm(double(reshape(pointLoc(i, j, :), [], 1)));
            if dist < minRange
                excludedLoc = [excludedLoc; reshape(pointLoc(i, j, :), [], 3)];
                excludedIntensity = [excludedIntensity; ptCloud.Intensity(i, j)];
                
                pointLoc(i, j, :) = nan;   
                pointInt(i, j) = nan;
            end
        end
    end
end


ptCloud = pointCloud(pointLoc, 'Intensity', ptCloud.Intensity);

% % Plotting
% figure; hold on;
% pcshow(excludedLoc, [1 1 1], 'MarkerSize', 100 );
% pcshow(ptCloud);

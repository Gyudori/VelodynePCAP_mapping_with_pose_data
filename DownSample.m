function sampledData = DownSample(arrayData, samplingInterval)

rowSize = ceil(size(arrayData, 1) / samplingInterval);

index = 0;
for samplingIndex = 1:samplingInterval:size(arrayData, 1)  
    index = index+1; 
    sampledData(index, :) = arrayData(samplingIndex, :);
end
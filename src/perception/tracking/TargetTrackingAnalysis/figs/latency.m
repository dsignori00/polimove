%% LATENCY FIGURE
figure('name','Detections - Latency', 'NumberTitle', 'off');

movingAverageWindowSeconds = 2;

validSensors = [];
for i = 1:numel(sensors)
    s = sensors{i}.s;
    hasValidStamp = any(isfinite(s.stamp(:))) && any(isfinite(s.sens_stamp(:)));
    hasValidMeasure = any(isfinite(s.x_rel(:))) || any(isfinite(s.y_rel(:))) || ...
        any(isfinite(s.x_map(:))) || any(isfinite(s.y_map(:)));

    if hasValidStamp && hasValidMeasure
        validSensors(end+1) = i; %#ok<SAGROW>
    end
end

tiledlayout(max(1, numel(validSensors)),1,'Padding','compact');

for i = validSensors
    ax(f) = nexttile([1,1]); f=f+1; hold on;
    s = sensors{i}.s;

    sensorStamp = s.sens_stamp;
    receiveStamp = s.stamp;
    if isvector(receiveStamp) && numel(receiveStamp) == size(sensorStamp,1)
        receiveStamp = repmat(receiveStamp(:), 1, size(sensorStamp,2));
    elseif ~isequal(size(receiveStamp), size(sensorStamp))
        error('Timestamp dimensions do not match for sensor %s.', sensors{i}.name);
    end

    latencies = receiveStamp - sensorStamp;
    validMeasure = ...
        isfinite(s.x_rel) | isfinite(s.y_rel) | ...
        isfinite(s.x_map) | isfinite(s.y_map);
    validLatency = ...
        isfinite(receiveStamp) & isfinite(sensorStamp) & validMeasure;
    latencies(~validLatency) = NaN;

    filterInputStamp = receiveStamp(validLatency);
    filterInputLatencies = latencies(validLatency);
    [sortedStamp, order] = sort(filterInputStamp);
    sortedLatencies = filterInputLatencies;
    sortedLatencies = sortedLatencies(order);

    [filterStamp, ~, stampGroup] = unique(sortedStamp);
    meanLatency = accumarray(stampGroup, sortedLatencies, [], @mean);
    filteredLatency = movmean(meanLatency, movingAverageWindowSeconds, ...
        'SamplePoints', filterStamp);

    maxDet = min(s.max_det, size(latencies,2));
    plot_detections(receiveStamp, latencies, maxDet, ...
        sensors{i}.col, sensors{i}.name, '-');
    plot(filterStamp, filteredLatency, ...
        'Color', 'k', ...
        'LineWidth', 2.5, ...
        'DisplayName', sprintf('moving average (%.1f s)', movingAverageWindowSeconds));
    ylabel(sprintf('%s [s]', sensors{i}.name)); legend show; grid on;
end
xlabel('timestamp [s]');

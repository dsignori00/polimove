function [correctMask, falsePositiveMask, validMask] = ...
        classify_lidar_detections(measurementStamp, measurementX, ...
        measurementY, groundTruthStamp, groundTruthX, groundTruthY, ...
        temporalGate, spatialGate)
%CLASSIFY_LIDAR_DETECTIONS Classify detections against ground truth.
% A detection is correct when its closest ground-truth sample is within the
% temporal gate and its map position is less than spatialGate meters away.

    validMask = isfinite(measurementStamp) & isfinite(measurementX) & ...
        isfinite(measurementY);
    correctMask = false(size(validMask));

    validGroundTruth = isfinite(groundTruthStamp) & ...
        isfinite(groundTruthX) & isfinite(groundTruthY);
    if ~any(validMask, 'all') || ~any(validGroundTruth, 'all')
        falsePositiveMask = validMask;
        return;
    end

    groundTruthStamp = double(groundTruthStamp(validGroundTruth));
    groundTruthX = groundTruthX(validGroundTruth);
    groundTruthY = groundTruthY(validGroundTruth);

    [groundTruthStamp, sortIdx] = sort(groundTruthStamp);
    groundTruthX = groundTruthX(sortIdx);
    groundTruthY = groundTruthY(sortIdx);
    [groundTruthStamp, uniqueIdx] = unique(groundTruthStamp, 'stable');
    groundTruthX = groundTruthX(uniqueIdx);
    groundTruthY = groundTruthY(uniqueIdx);

    validMeasurementIdx = find(validMask);
    queryStamp = double(measurementStamp(validMask));
    if isscalar(groundTruthStamp)
        closestGroundTruthIdx = ones(size(queryStamp));
    else
        closestGroundTruthIdx = interp1(groundTruthStamp, ...
            (1:numel(groundTruthStamp))', queryStamp, 'nearest', 'extrap');
    end

    timeError = abs(queryStamp - groundTruthStamp(closestGroundTruthIdx));
    positionError = hypot( ...
        measurementX(validMask) - groundTruthX(closestGroundTruthIdx), ...
        measurementY(validMask) - groundTruthY(closestGroundTruthIdx));
    isCorrect = timeError <= temporalGate & positionError < spatialGate;
    correctMask(validMeasurementIdx(isCorrect)) = true;
    falsePositiveMask = validMask & ~correctMask;
end

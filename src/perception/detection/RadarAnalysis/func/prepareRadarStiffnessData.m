function data = prepareRadarStiffnessData(rad,detection_stamp, ...
        acceleration_query_stamp,ego_stamp,ego_lateral_acceleration, ...
        gt_stamp,gt_x_rel,gt_y_rel,correct_mask,options)
%PREPARERADARSTIFFNESSDATA Align bearing error and ego lateral acceleration.

measurement_size = size(rad.x_rel);
detection_stamp = expandToSize(detection_stamp,measurement_size, ...
    'detection stamp');
acceleration_query_stamp = expandToSize( ...
    acceleration_query_stamp,measurement_size,'acceleration query stamp');
correct_mask = expandToSize(correct_mask,measurement_size, ...
    'correct-detection mask');

[gt_stamp,gt_x_rel,gt_y_rel] = sanitizeReference( ...
    gt_stamp,gt_x_rel,gt_y_rel,'ground truth');
[ego_stamp,ego_lateral_acceleration] = sanitizeReference( ...
    ego_stamp,ego_lateral_acceleration,[],'ego acceleration');

gt_x_interp = interp1(gt_stamp,gt_x_rel,detection_stamp,'linear',NaN);
gt_y_interp = interp1(gt_stamp,gt_y_rel,detection_stamp,'linear',NaN);
gt_nearest_stamp = interp1( ...
    gt_stamp,gt_stamp,detection_stamp,'nearest',NaN);

acceleration_interp = interp1(ego_stamp,ego_lateral_acceleration, ...
    acceleration_query_stamp,'linear',NaN);
ego_nearest_stamp = interp1(ego_stamp,ego_stamp, ...
    acceleration_query_stamp,'nearest',NaN);

measured_bearing = atan2d(rad.y_rel,rad.x_rel);
reference_bearing = atan2d(gt_y_interp,gt_x_interp);
bearing_difference = measured_bearing - reference_bearing;
angular_error = atan2d(sind(bearing_difference),cosd(bearing_difference));
range = hypot(rad.x_rel,rad.y_rel);

valid = logical(correct_mask) & ...
    isfinite(rad.x_rel) & isfinite(rad.y_rel) & ...
    isfinite(angular_error) & isfinite(acceleration_interp) & ...
    range >= options.MIN_RANGE & ...
    abs(detection_stamp - gt_nearest_stamp) <= options.MAX_TIME_GAP & ...
    abs(acceleration_query_stamp - ego_nearest_stamp) <= ...
        options.MAX_TIME_GAP;

data.lateral_acceleration_g = acceleration_interp/options.GRAVITY;
data.angular_error_deg = angular_error;
data.valid = valid;
data.front = valid & rad.x_rel > 0;
data.back = valid & rad.x_rel < 0;
end

function values = expandToSize(values,target_size,description)
if isequal(size(values),target_size)
    return;
end

if isvector(values) && numel(values) == target_size(1)
    values = repmat(values(:),1,target_size(2));
    return;
end

error('RadarAnalysis:InvalidStiffnessInputSize', ...
    '%s cannot be expanded to [%d %d].', ...
    description,target_size(1),target_size(2));
end

function [stamp,first_signal,second_signal] = sanitizeReference( ...
        stamp,first_signal,second_signal,description)
stamp = double(stamp(:));
first_signal = double(first_signal(:));

if isempty(second_signal)
    valid = isfinite(stamp) & isfinite(first_signal);
else
    second_signal = double(second_signal(:));
    valid = isfinite(stamp) & isfinite(first_signal) & ...
        isfinite(second_signal);
end

stamp = stamp(valid);
first_signal = first_signal(valid);
if ~isempty(second_signal)
    second_signal = second_signal(valid);
end

[stamp,unique_indices] = unique(stamp,'sorted');
first_signal = first_signal(unique_indices);
if ~isempty(second_signal)
    second_signal = second_signal(unique_indices);
end

if numel(stamp) < 2
    error('RadarAnalysis:InsufficientStiffnessReference', ...
        '%s needs at least two finite, unique timestamps.',description);
end
end

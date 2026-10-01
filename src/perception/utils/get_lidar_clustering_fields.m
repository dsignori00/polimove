function out = get_lidar_clustering_fields(in)
%GET_LIDAR_CLUSTERING_FIELDS Extract lidar clustering measurements from a log.

    detections = in.perception__lidar__clustering_detections;

    if isfield(detections, 'detections__sensor_stamp__tot')
        out.sens_stamp = detections.detections__sensor_stamp__tot;
    else
        out.sens_stamp = repmat(detections.sensor_stamp__tot, ...
            1, size(detections.detections__x_map, 2));
    end
    out.stamp = detections.stamp__tot;

    % Map-frame measurements.
    out.x_map = detections.detections__x_map;
    out.y_map = detections.detections__y_map;

    % CoG-frame measurements.
    out.x_rel = detections.detections__x_rel;
    out.y_rel = detections.detections__y_rel;

    % Velocity-related quantities are read directly from the measurement.
    measurementSize = size(out.x_map);
    out.rho_dot = optionalField(detections, 'detections__rho_dot', measurementSize);
    out.rho_dot_max = optionalField(detections, 'detections__rho_dot_max', measurementSize);
    out.vx = optionalField(detections, 'detections__vx', measurementSize);

    noMeasurement = (out.x_map == 0);
    fieldsToMask = {'sens_stamp', 'x_map', 'y_map', 'x_rel', 'y_rel', ...
        'rho_dot', 'rho_dot_max', 'vx'};
    for fieldIdx = 1:numel(fieldsToMask)
        fieldName = fieldsToMask{fieldIdx};
        values = out.(fieldName);
        values(noMeasurement) = NaN;
        out.(fieldName) = values;
    end

    out.count = size(out.x_map, 2) - sum(noMeasurement, 2);
    out.max_det = max(out.count, [], 'omitnan');
    if isempty(out.max_det)
        out.max_det = 0;
    end
end

function values = optionalField(inputStruct, fieldName, outputSize)
    if isfield(inputStruct, fieldName)
        values = inputStruct.(fieldName);
    else
        values = NaN(outputSize);
    end
end

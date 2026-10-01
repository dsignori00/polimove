function [lid_clust, rad_clust, cam_yolo, lid_pp, v2v] = load_perception(log)
% LOAD_PERCEPTION  Parse perception detections.
%
%   All detection structs are guaranteed to exist. Missing sources
%   are replaced with empty structs containing NaN fields.


%  Helper empty structs


emptyDetectionStruct = struct( ...
    'sens_stamp', NaN, ...
    'stamp', NaN, ...
    'x_rel', NaN, ...
    'y_rel', NaN, ...
    'z_rel', NaN, ...
    'yaw_rel', NaN, ...
    'x_map', NaN, ...
    'y_map', NaN, ...
    'z_map', NaN, ...
    'yaw_map', NaN, ...
    'count', NaN, ...
    'valid_yaw', NaN ...
);

emptyRadarStruct = emptyDetectionStruct;
emptyRadarStruct.rho_dot = NaN;

sens_stamp_field = "detections__sensor_stamp__tot";
% sens_stamp_field = "sensor_stamp__tot";

%  LIDAR CLUSTERING DETECTIONS

if isfield(log,'perception__lidar__clustering_detections')
    d = log.perception__lidar__clustering_detections;

    lid_clust.sens_stamp = d.(sens_stamp_field);
    lid_clust.stamp      = d.stamp__tot;

    lid_clust.x_rel   = replaceZeroWithNaN(d.detections__x_rel);
    lid_clust.y_rel   = replaceZeroWithNaN(d.detections__y_rel);
    lid_clust.z_rel   = replaceZeroWithNaN(d.detections__z_rel);
    lid_clust.yaw_rel = rad2deg(replaceZeroWithNaN(d.detections__yaw_rel));

    lid_clust.x_map   = replaceZeroWithNaN(d.detections__x_map);
    lid_clust.y_map   = replaceZeroWithNaN(d.detections__y_map);
    lid_clust.z_map   = replaceZeroWithNaN(d.detections__z_map);
    lid_clust.yaw_map = rad2deg(replaceZeroWithNaN(d.detections__yaw_map));
    lid_clust.valid_yaw = d.detections__valid_yaw;

    lid_clust.count    = getDetectionCount(d, lid_clust.x_rel);
    lid_clust.max_det = max(sum(~isnan(lid_clust.x_rel')));
else
    lid_clust = emptyDetectionStruct;
    lid_clust.max_det = 1;
end


%  RADAR CLUSTERING DETECTIONS

if isfield(log,'perception__radar__clustering_detections')
    d = log.perception__radar__clustering_detections;

    rad_clust.sens_stamp = d.(sens_stamp_field) ;
    rad_clust.stamp      = d.stamp__tot;

    rad_clust.x_rel   = replaceZeroWithNaN(d.detections__x_rel);
    rad_clust.y_rel   = replaceZeroWithNaN(d.detections__y_rel);
    rad_clust.z_rel   = replaceZeroWithNaN(d.detections__z_rel);
    rad_clust.yaw_rel = rad2deg(replaceZeroWithNaN(d.detections__yaw_rel));
    rad_clust.rho_dot = replaceZeroWithNaN(d.detections__rho_dot);

    rad_clust.x_map   = replaceZeroWithNaN(d.detections__x_map);
    rad_clust.y_map   = replaceZeroWithNaN(d.detections__y_map);
    rad_clust.z_map   = replaceZeroWithNaN(d.detections__z_map);
    rad_clust.yaw_map = rad2deg(replaceZeroWithNaN(d.detections__yaw_map));
    rad_clust.valid_yaw = d.detections__valid_yaw;

    rad_clust.count    = getDetectionCount(d, rad_clust.x_rel);
    rad_clust.max_det = max(sum(~isnan(rad_clust.x_rel')));
else
    rad_clust = emptyRadarStruct;
    rad_clust.max_det = 1;
end


%  CAMERA YOLO DETECTIONS

if isfield(log,'perception__camera__yolo_detections')
    d = log.perception__camera__yolo_detections;

    cam_yolo.sens_stamp = d.(sens_stamp_field)  ;
    cam_yolo.stamp      = d.stamp__tot;

    cam_yolo.x_rel   = replaceZeroWithNaN(d.detections__x_rel);
    cam_yolo.y_rel   = replaceZeroWithNaN(d.detections__y_rel);
    cam_yolo.z_rel   = replaceZeroWithNaN(d.detections__z_rel);
    cam_yolo.yaw_rel = rad2deg(replaceZeroWithNaN(d.detections__yaw_rel));

    cam_yolo.x_map   = replaceZeroWithNaN(d.detections__x_map);
    cam_yolo.y_map   = replaceZeroWithNaN(d.detections__y_map);
    cam_yolo.z_map   = replaceZeroWithNaN(d.detections__z_map);
    cam_yolo.yaw_map = rad2deg(replaceZeroWithNaN(d.detections__yaw_map));
    cam_yolo.valid_yaw = d.detections__valid_yaw;

    cam_yolo.count    = getDetectionCount(d, cam_yolo.x_rel);
    cam_yolo.max_det = max(sum(~isnan(cam_yolo.x_rel')));
else
    cam_yolo = emptyDetectionStruct;
    cam_yolo.max_det = 1;
end


%  LIDAR POINTPILLARS DETECTIONS

if isfield(log,'perception__lidar__pointpillars_detections')
    d = log.perception__lidar__pointpillars_detections;

    lid_pp.sens_stamp = d.(sens_stamp_field);
    lid_pp.stamp      = d.stamp__tot;

    lid_pp.x_rel   = replaceZeroWithNaN(d.detections__x_rel);
    lid_pp.y_rel   = replaceZeroWithNaN(d.detections__y_rel);
    lid_pp.z_rel   = replaceZeroWithNaN(d.detections__z_rel);
    lid_pp.yaw_rel = rad2deg(replaceZeroWithNaN(d.detections__yaw_rel));

    lid_pp.x_map   = replaceZeroWithNaN(d.detections__x_map);
    lid_pp.y_map   = replaceZeroWithNaN(d.detections__y_map);
    lid_pp.z_map   = replaceZeroWithNaN(d.detections__z_map);
    lid_pp.yaw_map = rad2deg(replaceZeroWithNaN(d.detections__yaw_map));
    lid_pp.valid_yaw = d.detections__valid_yaw;

    lid_pp.count    = getDetectionCount(d, lid_pp.x_rel);
    lid_pp.max_det = max(sum(~isnan(lid_pp.x_rel')));
else
    lid_pp = emptyDetectionStruct;
    lid_pp.max_det = 1;
end


%  V2V DETECTIONS

if isfield(log,'perception__v2v__detections')
    d = log.perception__v2v__detections;

    v2v.sens_stamp = d.(sens_stamp_field);
    v2v.stamp      = d.stamp__tot;

    v2v.x_rel   = replaceZeroWithNaN(d.detections__x_rel);
    v2v.y_rel   = replaceZeroWithNaN(d.detections__y_rel);
    v2v.z_rel   = replaceZeroWithNaN(d.detections__z_rel);
    v2v.yaw_rel = rad2deg(replaceZeroWithNaN(d.detections__yaw_rel));
    v2v.vx      = replaceZeroWithNaN(d.detections__vx);

    v2v.x_map   = replaceZeroWithNaN(d.detections__x_map);
    v2v.y_map   = replaceZeroWithNaN(d.detections__y_map);
    v2v.z_map   = replaceZeroWithNaN(d.detections__z_map);
    v2v.yaw_map = rad2deg(replaceZeroWithNaN(d.detections__yaw_map));
    v2v.valid_yaw = d.detections__valid_yaw;

    v2v.count   = getDetectionCount(d, v2v.x_rel);
    v2v.max_det = max(sum(~isnan(v2v.x_rel')));
else
    v2v = emptyDetectionStruct;
    v2v.max_det = 1;
end

%  NaN CHECK: Confirm at least one detection source contains data

isAllNaNStruct = @(s) all(structfun(@(x) ...
    (isnumeric(x) && all(isnan(x(:)))) || isempty(x), ...
    s, 'UniformOutput', true));

if all([
    isAllNaNStruct(lid_clust), ...
    isAllNaNStruct(rad_clust), ...
    isAllNaNStruct(cam_yolo), ...
    isAllNaNStruct(lid_pp), ...
    isAllNaNStruct(v2v)
])
    error('No detections found: all detection structures are empty or contain only NaN values.');
end

end % FUNCTION END


%  Helper function to replace zeros with NaN

function out = replaceZeroWithNaN(x)
    out = x;
    out(out == 0) = NaN;
end

function count = getDetectionCount(d, x_rel)
    if isfield(d, 'count')
        count = d.count;
    else
        count = sum(isfinite(x_rel), 2);
    end
end

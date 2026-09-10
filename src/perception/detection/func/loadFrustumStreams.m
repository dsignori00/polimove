function streams = loadFrustumStreams(log, camera_name)

    defs = { ...
        "bbox_points", "Frustum",  [0 1 0]; ...
        "filt_points", "Filtered", [0 0 1]; ...
        "car_points",  "BBox",     [1 0 0]  ...
    };

    streams = struct('name', {}, 'points', {}, 't', {}, 'color', {});

    for i = 1:size(defs, 1)

        topic = char("perception__camera__" + camera_name + "__" + defs{i,1});

        if ~isfield(log, topic)
            continue
        end

        streams(end+1).name = defs{i,2}; %#ok<AGROW>
        streams(end).points = pointCloud2TableToCellArray(log, topic);
        streams(end).t = double(log.(topic).bag_stamp(:));
        streams(end).color = defs{i,3};
    end
end
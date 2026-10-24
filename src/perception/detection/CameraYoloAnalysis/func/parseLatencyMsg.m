function latency = parseLatencyMsg(log, cam_names)
   

    n_cameras = numel(cam_names);
    latency = struct();
    for idx=1:n_cameras
        name = cam_names(idx);
        if isfield(log,'perception__camera__dbg__'+name+'__latency')
            l = log.('perception__camera__dbg__'+name+'__latency');
            latency.(name).stamp = replaceZeroWithNaN(l.stamp__tot);
            latency.(name).cam_enh = replaceZeroWithNaN(l.cam_enh_ms);
            latency.(name).cam_only = replaceZeroWithNaN(l.cam_only_ms);
            latency.(name).decode_img = replaceZeroWithNaN(l.decode_img_ms);
            latency.(name).decode_pcl = replaceZeroWithNaN(l.decode_pcl_ms);
            latency.(name).inference = replaceZeroWithNaN(l.inference_ms);
            latency.(name).delta_est_cam = replaceZeroWithNaN(l.delta_est_cam_ms);
            latency.(name).delta_lid_cam = replaceZeroWithNaN(l.delta_lid_cam_ms);
        end
        
    end

    if isempty(fieldnames(latency))
        latency = NaN;
    end

end

function out = replaceZeroWithNaN(x)
    out = double(x);
    out(out == 0) = NaN;
end
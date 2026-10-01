% PREPROCESSING OF MEASUREMENTS
for k = 1:numel(sensors)
    s = sensors{k}.s;

    % This analysis intentionally evaluates only the first detection.
    valid = isfinite(s.sens_stamp(:,1)) & isfinite(s.x_map(:,1));
    for l = fields
        field = l{1};
        s.(field) = s.(field)(valid,1);
    end

    if isfield(s,'rho_dot')
        s.rho_dot = s.rho_dot(valid,1);
    end

    s.yaw_rel = rad2deg(unwrap(deg2rad(s.yaw_rel)));
    s.max_det = 1;

    sensors{k}.s = s;  
end


% INTERPOLATION OF GT AND ERROR COMPUTATION
for k = 1:numel(sensors)
    s = sensors{k}.s;

    for l = interpFields
        s.([l{1} '_gt'])  = interp1(gt.stamp, gt.(l{1}), s.sens_stamp);
        s.([l{1} '_err']) = s.(l{1}) - s.([l{1} '_gt']);
    end

    if sensors{k}.has_rho_dot
        s.rho_dot_gt  = interp1(gt.stamp, gt.rho_dot, s.sens_stamp);
        s.rho_dot_err = s.rho_dot - s.rho_dot_gt;
    end

    s.yaw_map = unwrap_angle_smart_deg(s.yaw_map, s.yaw_map_gt);
    s.yaw_rel = unwrap_angle_smart_deg(s.yaw_rel, s.yaw_rel_gt);
    s.yaw_map_err = s.yaw_map - s.yaw_map_gt;
    s.yaw_rel_err = s.yaw_rel - s.yaw_rel_gt;

    sensors{k}.s = s;  
end

% STATISTICS COMPUTATION
for k = 1:numel(sensors)
    s = sensors{k}.s;

    ass = hypot(s.x_map_err, s.y_map_err) < err_thr;

    stats = {'x_rel','y_rel','yaw_map'};
    for l = stats
        s.([l{1} '_std'])  = std(s.([l{1} '_err'])(ass));
        s.([l{1} '_mean']) = mean(s.([l{1} '_err'])(ass));
    end

    if sensors{k}.has_rho_dot
        s.rho_dot_std  = std(s.rho_dot_err(ass));
        s.rho_dot_mean = mean(s.rho_dot_err(ass));
    end

    [s.x_ellipse, s.y_ellipse] = calculate_ellipse( ...
        s.x_rel_std, s.y_rel_std, s.x_rel_mean, s.y_rel_mean);

    sensors{k}.s = s;  
end

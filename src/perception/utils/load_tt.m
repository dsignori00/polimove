function tt = load_tt(log)
    if isfield(log, "perception__opponents")
        tt.stamp = log.perception__opponents.stamp__tot;
        % relative
        tt.x_rel = log.perception__opponents.opponents__x_rel;
        tt.y_rel = log.perception__opponents.opponents__y_rel;
        tt.rho_dot = log.perception__opponents.opponents__rho_dot;
        tt.yaw_rel = rad2deg(wrapToPi(log.perception__opponents.opponents__psi_rel));
        tt.x_rel(tt.x_rel==0)=nan;
        tt.y_rel(tt.y_rel==0)=nan;
        tt.rho_dot(tt.rho_dot==0)=nan;
        tt.yaw_rel(tt.yaw_rel==0)=nan;
        % map
        tt.x_map = log.perception__opponents.opponents__x_geom;
        tt.y_map = log.perception__opponents.opponents__y_geom;
        tt.vx = log.perception__opponents.opponents__vx;
        tt.yaw_map = rad2deg(wrapToPi(log.perception__opponents.opponents__psi));
        tt.ax = log.perception__opponents.opponents__ax;
        tt.covariance = valid_covariance(log.perception__opponents.opponents__ekf_p);
        tt.count = log.perception__opponents.count;
        tt.max_opp = max(tt.count);
        empty_buffer = nan(size(tt.x_map));
        tt.buffer.lid_clust = get_measure_buffer( ...
            log.perception__opponents, "opponents__lid_clust_meas", empty_buffer);
        tt.buffer.rad_clust = get_measure_buffer( ...
            log.perception__opponents, "opponents__rad_clust_meas", empty_buffer);
        tt.buffer.cam_yolo = get_measure_buffer( ...
            log.perception__opponents, "opponents__cam_yolo_meas", empty_buffer);
        tt.buffer.lid_pp = get_measure_buffer( ...
            log.perception__opponents, "opponents__lid_pp_meas", empty_buffer);
        tt.buffer.v2v = get_measure_buffer( ...
            log.perception__opponents, "opponents__v2v_meas", empty_buffer);
        tt.x_map(tt.x_map==0)=nan;
        tt.y_map(tt.y_map==0)=nan;
        tt.vx(tt.vx==0)=nan;
        tt.yaw_map(tt.yaw_map==0)=nan;
        tt.ax(tt.ax==0)=nan;
        if isfield(log.perception__opponents,"opponents__yaw_rate")
            tt.yaw_rate = rad2deg(log.perception__opponents.opponents__yaw_rate);
            tt.yaw_rate(tt.yaw_rate==0)=nan;
        elseif isfield(log.perception__opponents,"opponents__psi_dot")
            tt.yaw_rate = rad2deg(log.perception__opponents.opponents__psi_dot);
            tt.yaw_rate(tt.yaw_rate==0)=nan;
        end
        if isfield(log.perception__opponents,"opponents__ax_dot")
            tt.ax_dot = log.perception__opponents.opponents__ax_dot;
            tt.ax_dot(tt.ax_dot==0)=nan;
        end
        if isfield(log.perception__opponents,"opponents__ff_flag")
            tt.ff_flag = log.perception__opponents.opponents__ff_flag;
        end
        if isfield(log.perception__opponents,"opponents__probabilities")
            tt.imm.probabilities = log.perception__opponents.opponents__probabilities;
            tt.imm.likelihoods = log.perception__opponents.opponents__likelihoods;
            tt.imm.residuals = log.perception__opponents.opponents__residuals;
        else
            tt.imm.probabilities = nan(size(tt.stamp));
            tt.imm.likelihoods = nan(size(tt.stamp));
            tt.imm.residuals = nan(size(tt.stamp));
        end
        % associated measures
        if isfield(log.perception__opponents,"opponents__meas_count")
            tt.measures.count = log.perception__opponents.opponents__meas_count;
            tt.measures.source = log.perception__opponents.opponents__meas_source_type;
            tt.measures.stamp = log.perception__opponents.opponents__meas_stamp;
            tt.measures.x_map = log.perception__opponents.opponents__meas_x_map;
            tt.measures.y_map = log.perception__opponents.opponents__meas_y_map;
            tt.measures.stamp(tt.measures.stamp==0)=nan;
            tt.measures.x_map(tt.measures.x_map==0)=nan;
            tt.measures.y_map(tt.measures.y_map==0)=nan;
            tt.measures.stamp = tt.measures.stamp - log.time_offset_nsec*1e-9;
        end
    else
        error('No target tracking data found in the log.');
    end
end

function buffer = get_measure_buffer(opponents, field_name, empty_buffer)
    if isfield(opponents, field_name)
        buffer = opponents.(field_name);
    else
        buffer = empty_buffer;
    end
end

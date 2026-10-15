%% DETECTION TIME SERIES

detection_figure = figure('Name','Detections','NumberTitle','off');
detection_tabs = uitabgroup(detection_figure);

% ---------------- COG ----------------
cog_tab = uitab(detection_tabs,'Title','CoG');
cog_layout = tiledlayout(cog_tab,2,1, ...
    'Padding','compact','TileSpacing','compact');

ax(f) = nexttile(cog_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.x_rel,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.x_rel,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.x_rel,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('x rel [m]'); xlim(x_lim); ylim([-200 200]); legend show;

ax(f) = nexttile(cog_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.y_rel,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.y_rel,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.y_rel,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('y rel [m]'); xlabel('timestamp [s]');
xlim(x_lim); ylim([-100 100]); legend show;

% ---------------- RANGE ----------------
range_tab = uitab(detection_tabs,'Title','Range');
range_layout = tiledlayout(range_tab,1,1, ...
    'Padding','compact','TileSpacing','compact');

ax(f) = nexttile(range_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.range,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.range,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.rho,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('range [m]'); xlabel('timestamp [s]');
xlim(x_lim); ylim([0 200]); legend show;

% ---------------- MAP ----------------
map_tab = uitab(detection_tabs,'Title','Map');
map_layout = tiledlayout(map_tab,2,1, ...
    'Padding','compact','TileSpacing','compact');

ax(f) = nexttile(map_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.x_map,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.x_map,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.x_map,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('x map [m]'); xlim(x_lim); legend show;

ax(f) = nexttile(map_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.y_map,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.y_map,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.y_map,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('y map [m]'); xlabel('timestamp [s]');
xlim(x_lim); legend show;

% ---------------- SPEED ----------------
speed_tab = uitab(detection_tabs,'Title','Speed');
speed_layout = tiledlayout(speed_tab,2,1, ...
    'Padding','compact','TileSpacing','compact');

ax(f) = nexttile(speed_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.rho_dot,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.rho_dot,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.rho_dot,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('rho dot [m/s]'); xlim(x_lim); legend show;

ax(f) = nexttile(speed_layout); f = f + 1; hold on;
plotClassifiedDetections(rad1.sens_stamp,rad1.vx,rad1.max_det, ...
    col.radar,NAME_1,sz,ground_truth,rad1.correct,rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp,rad2.vx,rad2.max_det, ...
        col.radar2,NAME_2,sz,ground_truth,rad2.correct, ...
        rad2.false_positive);
end
if ground_truth
    plot(gt_timestamp,gt.speed,'Color',col.ref,'DisplayName','gt');
end
grid on; ylabel('v_x [m/s]'); xlabel('timestamp [s]');
xlim(x_lim); ylim([0 inf]); legend show;

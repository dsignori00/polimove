%% STATE FIGURE MAP
time_series_fig = figure('Name','Time series','NumberTitle','off');
time_series_tabs = uitabgroup(time_series_fig);
time_series_map_tab = uitab(time_series_tabs,'Title','Map');
map_layout = tiledlayout(time_series_map_tab,3,1,'Padding','compact');

% pos x
ax(f) = nexttile(map_layout); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.x_map, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot(gt.stamp,gt.x_map,'Color',col.ref,'DisplayName','gt'); 
grid on; ylabel('x map [m]'); legend show;

% pos y
ax(f) = nexttile(map_layout); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.y_map, s.max_det, sensors{i}.col, sensors{i}.name);
end

plot(gt.stamp,gt.y_map,'Color',col.ref,'DisplayName','gt'); 
grid on; ylabel('y map [m]'); legend show;

% yaw
ax(f) = nexttile(map_layout); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.yaw_map, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot(gt.stamp,gt.yaw_map,'Color',col.ref,'DisplayName','gt'); 
grid on; ylabel('yaw [deg]'); legend show; xlabel('timestamp [s]');


%% STATE FIGURE REL
time_series_cog_tab = uitab(time_series_tabs,'Title','CoG');
cog_layout = tiledlayout(time_series_cog_tab, ...
    2 + double(use_tt),1,'Padding','compact');

% pos x
ax(f) = nexttile(cog_layout); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.x_rel, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','gt'); 
grid on; ylabel('x rel [m]'); legend show;

% pos y
ax(f) = nexttile(cog_layout); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.y_rel, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','gt'); 
grid on; ylabel('y rel [m]'); legend show; ylim([-100 100]);

% count
if use_tt
    ax(f) = nexttile(cog_layout); f=f+1; hold on;
    plot_area(tt.stamp, tt.buffer.rad_clust, tt.max_opp, col.radar, 'Rad Clust');
    plot_area(tt.stamp, tt.buffer.lid_pp,    tt.max_opp, col.pp,    'Lid PP');
    plot_area(tt.stamp, tt.buffer.lid_clust, tt.max_opp, col.lidar,'Lid Clust');
    plot_area(tt.stamp, tt.buffer.cam_yolo,  tt.max_opp, col.camera,'Camera');
    grid on; ylabel('count'); legend show; xlabel('timestamp [s]');
else
    xlabel('timestamp [s]');
end

%% RANGE
time_series_range_tab = uitab(time_series_tabs,'Title','Range');
range_layout = tiledlayout(time_series_range_tab,2,1,'Padding','compact');

% rho 
ax(f) = nexttile(range_layout); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    s.range = sqrt(s.x_rel.^2 + s.y_rel.^2);
    plot_detections(s.sens_stamp, s.range, s.max_det, sensors{i}.col, sensors{i}.name);
end

if(use_sim_ref); gt.rho = sqrt(gt.x_rel.^2 + gt.y_rel.^2); end
plot(gt.stamp, gt.rho, 'Color',col.ref,'DisplayName','gt'); 
grid on; ylabel('range [m]'); ylim([0 200]); legend show;

% rho dot
ax(f) = nexttile(range_layout); f=f+1; hold on;
radarIdx = find(cellfun(@(sensor) sensor.has_rho_dot, sensors), 1);
if ~isempty(radarIdx)
    radar = sensors{radarIdx}.s;
    plot_detections(radar.sens_stamp, radar.rho_dot, radar.max_det, sensors{radarIdx}.col, sensors{radarIdx}.name);
end
plot(gt.stamp, gt.rho_dot, 'Color',col.ref,'DisplayName','gt')
grid on; ylabel('rho dot [m/s]'); legend show; ylim([-50 50]); xlabel('timestamp [s]');


%% STATE FIGURE MAP
figure('name', 'Filter - Map', 'NumberTitle', 'off');
tiledlayout(3,1,'Padding','compact');

% pos x
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.x_map, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot_tt(tt.stamp, tt.x_map, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.x_map, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.x_map, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp,gt.x_map,'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('x map [m]'); legend show;

% pos y
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.y_map, s.max_det, sensors{i}.col, sensors{i}.name);
end

plot_tt(tt.stamp, tt.y_map, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.y_map, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.y_map, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp,gt.y_map,'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('y map [m]'); legend show;

% yaw
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, unwrap_pi(s.yaw_map), s.max_det, sensors{i}.col, sensors{i}.name);
end
    
plot_tt(tt.stamp, tt.yaw_map, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.yaw_map, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.yaw_map, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp,gt.yaw_map,'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('yaw [deg]'); xlabel('timestamp [s]'); legend show;

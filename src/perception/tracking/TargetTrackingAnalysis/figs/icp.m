%% SPEED AND ACC
figure('name', 'Filter - Pointpillars Icp', 'NumberTitle', 'off');
tiledlayout(3,1,'Padding','compact');

% rho 
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    s.range = sqrt(s.x_rel.^2 + s.y_rel.^2);
    plot_detections(s.sens_stamp, s.range, s.max_det, sensors{i}.col, sensors{i}.name);
end

tt.range = sqrt(tt.x_rel.^2 + tt.y_rel.^2);
plot_tt(tt.stamp, tt.range, tt.max_opp, col.tt, name1);

if(compare) 
    tt2.range = sqrt(tt2.x_rel.^2 + tt2.y_rel.^2);
    plot_tt(tt2.stamp, tt2.range, tt2.max_opp, col.tt2, name2);
end
if(compare2)
    tt3.range = sqrt(tt3.x_rel.^2 + tt3.y_rel.^2);
    plot_tt(tt3.stamp, tt3.range, tt3.max_opp, col.tt3, name3);
end

if(use_ref || use_sim_ref)
    if(use_sim_ref); gt.rho = sqrt(gt.x_rel.^2 + gt.y_rel.^2); end
    plot(gt.stamp, gt.rho, 'Color',col.ref,'DisplayName','gt'); 
end
grid on; ylabel('range [m]'); ylim([0 200]); legend show;

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

% valid yaw
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i =1:lid_pp.max_det
    plot(lid_pp.sens_stamp, lid_pp.valid_yaw(:,i), 'DisplayName', sprintf('Track %d', i));
end
grid on; ylabel('valid yaw'); xlabel('timestamp [s]'); legend show;

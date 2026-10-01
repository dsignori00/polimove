%% RANGE
figure('name', 'Filter - Range', 'NumberTitle', 'off');
tiledlayout(2,1,'Padding','compact');

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

% rho dot
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    if isfield(s, 'rho_dot')
        plot_detections(s.sens_stamp, s.rho_dot, s.max_det, sensors{i}.col, sensors{i}.name);
    end
end
plot_tt(tt.stamp, tt.rho_dot, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.rho_dot, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.rho_dot, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp, gt.rho_dot, 'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('rho dot [m/s]'); legend show;

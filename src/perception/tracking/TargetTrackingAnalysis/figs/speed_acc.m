%% SPEED AND ACC
figure('name', 'Filter - Speed Acc', 'NumberTitle', 'off');
tiledlayout(3,1,'Padding','compact');

% vx
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    if isfield(s, 'vx')
        plot_detections(s.sens_stamp, s.vx, s.max_det, sensors{i}.col, sensors{i}.name);
    end
end
plot_tt(tt.stamp, tt.vx, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.vx, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.vx, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp,gt.vx,'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('vx [m/s]'); ylim([0 inf]); legend show;

% ax
ax(f) = nexttile([1,1]); f=f+1; hold on;
plot_tt(tt.stamp, tt.ax, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.ax, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.ax, tt3.max_opp, col.tt3, name3); end
if(use_sim_ref || use_ref); plot(gt.stamp,gt.ax,'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('ax [m/s$^2$]'); legend show; 

% ax_dot
ax(f) = nexttile([1,1]); f=f+1; hold on;
if(isfield(tt,'ax_dot')); plot_tt(tt.stamp, tt.ax_dot, tt.max_opp, col.tt, name1); end
if(compare && isfield(tt2,'ax_dot')); plot_tt(tt2.stamp, tt2.ax_dot, tt2.max_opp, col.tt2, name2); end
if(compare2 && isfield(tt3,'ax_dot')); plot_tt(tt3.stamp, tt3.ax_dot, tt3.max_opp, col.tt3, name3); end
grid on; ylabel('jerk [m/s$^3$]'); legend show; xlabel('timestamp [s]');

% % yaw rate
% ax(f) = nexttile([1,1]); f=f+1; hold on;
% if(isfield(tt,'yaw_rate')); plot_tt(tt.stamp, tt.yaw_rate, tt.max_opp, col.tt, name1); end
% if(compare && isfield(tt2,'yaw_rate')); plot_tt(tt2.stamp, tt2.yaw_rate, tt2.max_opp, col.tt2, name2); end
% if(compare2 && isfield(tt3,'yaw_rate')); plot_tt(tt3.stamp, tt3.yaw_rate, tt3.max_opp, col.tt3, name3); end
% if(use_sim_ref || use_ref); plot(gt.stamp,gt.yaw_rate,'Color',col.ref,'DisplayName','gt'); end
% grid on; ylabel('yaw rate [deg/s]'); legend show; xlabel('timestamp [s]');

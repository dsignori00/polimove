%% STATE FIGURE REL
if exist('state_cog_parent','var') && isgraphics(state_cog_parent,'uitab')
    state_cog_layout_parent = state_cog_parent;
else
    state_cog_figure = figure( ...
        'Name','Filter - CoG','NumberTitle','off','Color','w');
    state_cog_layout_parent = state_cog_figure;
end
state_cog_layout = tiledlayout(state_cog_layout_parent,3,1, ...
    'Padding','compact','TileSpacing','compact');

% pos x
ax(f) = nexttile(state_cog_layout,[1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.x_rel, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot_tt(tt.stamp, tt.x_rel, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.x_rel, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.x_rel, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('x rel [m]'); legend show;

% pos y
ax(f) = nexttile(state_cog_layout,[1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    s = sensors{i}.s;
    plot_detections(s.sens_stamp, s.y_rel, s.max_det, sensors{i}.col, sensors{i}.name);
end
plot_tt(tt.stamp, tt.y_rel, tt.max_opp, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.y_rel, tt2.max_opp, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.y_rel, tt3.max_opp, col.tt3, name3); end
if(use_ref || use_sim_ref); plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','gt'); end
grid on; ylabel('y rel [m]'); legend show;

% rho dot
ax(f) = nexttile(state_cog_layout,[1,1]); f=f+1; hold on;
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
grid on; ylabel('rho dot [m/s]'); xlabel('timestamp [s]'); legend show;

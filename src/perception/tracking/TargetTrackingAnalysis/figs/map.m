%#ok<*UNRCH>
%#ok<*INUSD>

%% MAP
fig = figure('name','MAP', 'NumberTitle', 'off');

% Button
c = c + 1;
b(c) = uicontrol('Style','pushbutton', ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@refreshTimeButtonPushed);

function refreshTimeButtonPushed(src, event)

    % --- fetch from base ---
    ax        = evalin('base','ax');
    traj_db  = evalin('base','trajDatabase');
    sensors  = evalin('base','sensors');
    col      = evalin('base','col');
    tt       = evalin('base','tt');
    name1    = evalin('base','name1');

    use_ref      = evalin('base','use_ref');
    use_sim_ref  = evalin('base','use_sim_ref');
    compare      = evalin('base','compare');
    compare2     = evalin('base','compare2');

    if compare
        tt2   = evalin('base','tt2');
        name2 = evalin('base','name2');
    end
    if compare2
        tt3   = evalin('base','tt3');
        name3 = evalin('base','name3');
    end
    if use_ref || use_sim_ref
        gt = evalin('base','gt');
    end

    % --- time window ---
    t_lim = xlim(ax(1));

    [t1_tt, tend_tt] = timeWindowIdx(tt.stamp, t_lim);
    if compare
        [t1_tt2, tend_tt2] = timeWindowIdx(tt2.stamp, t_lim);
    end
    if use_ref || use_sim_ref
        [t1_gt, tend_gt] = timeWindowIdx(gt.stamp, t_lim);
    end
    if compare2
        [t1_tt3, tend_tt3] = timeWindowIdx(tt3.stamp, t_lim);
    end

    % --- reset axes ---
    subplot(1,1,1); cla reset; hold on;
    grid on;
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');
    L = 3.0;

    % --- track boundaries ---
    id_left  = numel(traj_db) - 2;
    id_right = numel(traj_db) - 1;

    plot(traj_db(id_left).x,  traj_db(id_left).y,  'k','LineWidth',1,'HandleVisibility','off');
    plot(traj_db(id_right).x, traj_db(id_right).y, 'k','LineWidth',1,'HandleVisibility','off');

    % --- sensor detections ---
    for i = 1:numel(sensors)
        s = sensors{i}.s;

        valid = ...
            isfinite(s.sens_stamp) & ...
            s.sens_stamp >= t_lim(1) & ...
            s.sens_stamp <= t_lim(2) & ...
            isfinite(s.x_map) & ...
            isfinite(s.y_map);

        plot( ...
            s.x_map(valid), ...
            s.y_map(valid), ...
            '.', ...
            'MarkerSize', 20, ...
            'Color', sensors{i}.col, ...
            'DisplayName', sensors{i}.name);
    end



    % --- tracked targets ---
    plot(tt.x_map(t1_tt:tend_tt,1:tt.max_opp),tt.y_map(t1_tt:tend_tt,1:tt.max_opp),'Color',col.tt,'HandleVisibility','off');
    % quiver( ...
    %     tt.x_map(t1_tt:tend_tt,1:tt.max_opp), ...
    %     tt.y_map(t1_tt:tend_tt,1:tt.max_opp), ...
    %     L*cos(deg2rad(tt.yaw_map(t1_tt:tend_tt,1:tt.max_opp))), ...
    %     L*sin(deg2rad(tt.yaw_map(t1_tt:tend_tt,1:tt.max_opp))), ...
    %     0, ...
    %     'Color', col.tt, ...
    %     'HandleVisibility','off', ...
    %     'LineWidth', 2.5);
    plot_tt(NaN,NaN,1,col.tt,name1);

    if compare
        plot(tt2.x_map(t1_tt2:tend_tt2,1:tt2.max_opp),tt2.y_map(t1_tt2:tend_tt2,1:tt2.max_opp),'Color',col.tt2,'HandleVisibility','off');
        plot_tt(NaN,NaN,1,col.tt2,name2);
    end
    if compare2
        plot(tt3.x_map(t1_tt3:tend_tt3,1:tt3.max_opp),tt3.y_map(t1_tt3:tend_tt3,1:tt3.max_opp),'Color',col.tt3,'HandleVisibility','off');
        plot_tt(NaN,NaN,1,col.tt3,name3);
    end

    % --- ground truth ---
    if use_ref || use_sim_ref
        plot( ...
            gt.x_map(t1_gt:tend_gt), ...
            gt.y_map(t1_gt:tend_gt), ...
            'Color', col.ref, ...
            'DisplayName','gt');
    end

    legend show
end

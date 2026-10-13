%#ok<*UNRCH>
%#ok<*INUSD>

%% MAP
fig = figure('name','MAP', 'NumberTitle','off');

% Button
b = b + 1;
c(b) = uicontrol('Style','pushbutton', ...
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
    gt = evalin('base','gt');

    % --- time window ---
    t_lim = xlim(ax(1));
    [t1_gt, tend_gt] = timeWindowIdx(gt.stamp, t_lim);

    % --- reset ax ---
    subplot(1,1,1); cla reset; hold on;
    grid on;
    axis equal;
    xlabel('x [m]');
    ylabel('y [m]');

    % --- track boundaries ---
    id_left  = numel(traj_db) - 2;
    id_right = numel(traj_db) - 1;

    plot(traj_db(id_left).x,  traj_db(id_left).y,  'k','LineWidth',1,'HandleVisibility','off');
    plot(traj_db(id_right).x, traj_db(id_right).y, 'k','LineWidth',1,'HandleVisibility','off');

    % --- sensor detections ---
    for i = 1:numel(sensors)
        s = sensors{i}.s;
        [i1, i2] = timeWindowIdx(s.sens_stamp, t_lim);

        plot( ...
            s.x_map(i1:i2), ...
            s.y_map(i1:i2), ...
            '.', ...
            'MarkerSize', 20, ...
            'Color', sensors{i}.col, ...
            'DisplayName', sensors{i}.name);
    end

    % --- ground truth ---
    plot( ...
        gt.x_map(t1_gt:tend_gt), ...
        gt.y_map(t1_gt:tend_gt), ...
        'Color', col.ref, ...
        'DisplayName','gt');

    legend show
end

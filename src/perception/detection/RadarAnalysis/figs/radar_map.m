%% MAP

map_figure = figure('Name','Map','NumberTitle','off');
uicontrol('Style','pushbutton', ...
    'Parent',map_figure, ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@refreshTimeButtonPushed);

function refreshTimeButtonPushed(src,~)
    ax = evalin('base','ax');
    traj_db = evalin('base','trajDatabase');
    ground_truth = evalin('base','ground_truth');
    compare = evalin('base','compare');
    col = evalin('base','col');
    rad1 = evalin('base','rad1');
    bridge_position = evalin('base','BRIDGE_POSITION');
    primary_name = evalin('base','NAME_1');
    comparison_name = evalin('base','NAME_2');
    statistics_box_position = evalin('base','STATISTICS_BOX_POSITION');
    if ground_truth
        gt = evalin('base','gt');
        gt_timestamp = evalin('base','gt_timestamp');
    end
    if compare
        rad2 = evalin('base','rad2');
        rad2_sens_stamp = evalin('base','rad2_sens_stamp');
    end

    time_limits = xlim(ax(1));
    rad1_time_idx = rad1.sens_stamp > time_limits(1) & ...
        rad1.sens_stamp < time_limits(2);
    if compare
        rad2_time_idx = rad2_sens_stamp > time_limits(1) & ...
            rad2_sens_stamp < time_limits(2);
    end
    if ground_truth
        gt_idx = gt_timestamp > time_limits(1) & ...
            gt_timestamp < time_limits(2);
    end

    figure_handle = ancestor(src,'figure');
    map_axes = subplot(1,1,1,'Parent',figure_handle);
    cla(map_axes,'reset');
    title(map_axes,'map');
    hold(map_axes,'on');
    grid(map_axes,'on');
    xlabel(map_axes,'x [m]');
    ylabel(map_axes,'y [m]');
    axis(map_axes,'equal');

    id_left = numel(traj_db) - 2;
    id_right = numel(traj_db) - 1;
    plot(map_axes,traj_db(id_left).x,traj_db(id_left).y, ...
        'Color','k','LineWidth',1,'HandleVisibility','off');
    plot(map_axes,traj_db(id_right).x,traj_db(id_right).y, ...
        'Color','k','LineWidth',1,'HandleVisibility','off');

    if ground_truth
        rad1_tp_idx = rad1_time_idx & rad1.correct;
        rad1_fp_idx = rad1_time_idx & rad1.false_positive;
        scatter(map_axes,rad1.x_map(rad1_tp_idx),rad1.y_map(rad1_tp_idx), ...
            20,'filled','MarkerEdgeColor',col.radar, ...
            'MarkerFaceColor',col.radar, ...
            'DisplayName',primary_name + " - TP");
        scatter(map_axes,rad1.x_map(rad1_fp_idx),rad1.y_map(rad1_fp_idx), ...
            20,'o','MarkerEdgeColor',col.radar, ...
            'MarkerFaceColor','none', ...
            'DisplayName',primary_name + " - FP");
    else
        rad1_x_map = rad1.x_map(rad1_time_idx,:);
        rad1_y_map = rad1.y_map(rad1_time_idx,:);
        scatter(map_axes,rad1_x_map(:),rad1_y_map(:),20,'filled', ...
            'MarkerEdgeColor',col.radar,'MarkerFaceColor',col.radar, ...
            'DisplayName',primary_name);
    end

    if compare
        if ground_truth
            rad2_tp_idx = rad2_time_idx & rad2.correct;
            rad2_fp_idx = rad2_time_idx & rad2.false_positive;
            scatter(map_axes,rad2.x_map(rad2_tp_idx), ...
                rad2.y_map(rad2_tp_idx),20,'filled', ...
                'MarkerEdgeColor',col.radar2, ...
                'MarkerFaceColor',col.radar2, ...
                'DisplayName',comparison_name + " - TP");
            scatter(map_axes,rad2.x_map(rad2_fp_idx), ...
                rad2.y_map(rad2_fp_idx),20,'o', ...
                'MarkerEdgeColor',col.radar2, ...
                'MarkerFaceColor','none', ...
                'DisplayName',comparison_name + " - FP");
        else
            rad2_x_map = rad2.x_map(rad2_time_idx,:);
            rad2_y_map = rad2.y_map(rad2_time_idx,:);
            scatter(map_axes,rad2_x_map(:),rad2_y_map(:),20,'filled', ...
                'MarkerEdgeColor',col.radar2, ...
                'MarkerFaceColor',col.radar2, ...
                'DisplayName',comparison_name);
        end
    end

    if ground_truth
        plot(map_axes,gt.x_map(gt_idx),gt.y_map(gt_idx), ...
            'Color',col.ref,'DisplayName','Ground truth');
    end

    circle_angle = linspace(0,2*pi,64);
    for bridge_idx = 1:numel(bridge_position.X_MAP)
        handle_visibility = 'off';
        if bridge_idx == 1
            handle_visibility = 'on';
        end

        patch(map_axes, ...
            bridge_position.X_MAP(bridge_idx) + ...
                bridge_position.THRESHOLD*cos(circle_angle), ...
            bridge_position.Y_MAP(bridge_idx) + ...
                bridge_position.THRESHOLD*sin(circle_angle), ...
            'red','FaceAlpha',0.3,'EdgeColor','black', ...
            'HandleVisibility',handle_visibility, ...
            'DisplayName','Bridge positions','PickableParts','all', ...
            'HitTest','on', ...
            'ButtonDownFcn',@(circle,event) ...
                startBridgeDrag(circle,event,bridge_idx));
    end

    primary_stats.stamp = rad1.sens_stamp;
    primary_stats.valid = rad1.valid_detection;
    primary_stats.correct = rad1.correct;
    primary_stats.false_positive = rad1.false_positive;
    if compare
        comparison_stats.stamp = rad2_sens_stamp;
        comparison_stats.valid = rad2.valid_detection;
        comparison_stats.correct = rad2.correct;
        comparison_stats.false_positive = rad2.false_positive;
    else
        comparison_stats = struct();
    end

    map_statistics_box = findall(figure_handle, ...
        'Tag','MapDetectionStatistics');
    if isempty(map_statistics_box)
        map_statistics_box = annotation(figure_handle,'textbox', ...
            statistics_box_position,'Tag','MapDetectionStatistics', ...
            'FitBoxToText','off','BackgroundColor','white', ...
            'EdgeColor','black','Margin',4, ...
            'VerticalAlignment','top','Interpreter','none');
    end
    map_statistics_box.String = formatCombinedDetectionStatistics( ...
        primary_stats,primary_name,ground_truth,compare, ...
        comparison_stats,comparison_name,time_limits);

    legend(map_axes,'show');
end

function startBridgeDrag(circle,~,bridge_idx)
    fig = ancestor(circle,'figure');
    if ~strcmp(fig.SelectionType,'normal')
        return;
    end

    ax = ancestor(circle,'axes');
    bridge_position = evalin('base','BRIDGE_POSITION');
    center = [bridge_position.X_MAP(bridge_idx), ...
        bridge_position.Y_MAP(bridge_idx)];
    cursor_position = ax.CurrentPoint(1,1:2);

    drag_state.circle = circle;
    drag_state.axes = ax;
    drag_state.bridge_idx = bridge_idx;
    drag_state.cursor_offset = center - cursor_position;
    drag_state.circle_x_offset = circle.XData - center(1);
    drag_state.circle_y_offset = circle.YData - center(2);
    drag_state.center = center;

    setappdata(fig,'BridgeDragState',drag_state);
    fig.Pointer = 'fleur';
    fig.WindowButtonMotionFcn = @moveBridgeCircle;
    fig.WindowButtonUpFcn = @finishBridgeDrag;
end

function moveBridgeCircle(fig,~)
    if ~isappdata(fig,'BridgeDragState')
        return;
    end

    drag_state = getappdata(fig,'BridgeDragState');
    cursor_position = drag_state.axes.CurrentPoint(1,1:2);
    drag_state.center = cursor_position + drag_state.cursor_offset;

    drag_state.circle.XData = drag_state.center(1) + ...
        drag_state.circle_x_offset;
    drag_state.circle.YData = drag_state.center(2) + ...
        drag_state.circle_y_offset;
    setappdata(fig,'BridgeDragState',drag_state);
    drawnow limitrate nocallbacks;
end

function finishBridgeDrag(fig,~)
    if ~isappdata(fig,'BridgeDragState')
        return;
    end

    moveBridgeCircle(fig,[]);
    drag_state = getappdata(fig,'BridgeDragState');

    bridge_position = evalin('base','BRIDGE_POSITION');
    bridge_position.X_MAP(drag_state.bridge_idx) = drag_state.center(1);
    bridge_position.Y_MAP(drag_state.bridge_idx) = drag_state.center(2);
    assignin('base','BRIDGE_POSITION',bridge_position);

    fprintf('\nBridge circle %d moved:\n',drag_state.bridge_idx);
    fprintf('BRIDGE_POSITION.X_MAP(%d) = %.3f;\n', ...
        drag_state.bridge_idx,drag_state.center(1));
    fprintf('BRIDGE_POSITION.Y_MAP(%d) = %.3f;\n', ...
        drag_state.bridge_idx,drag_state.center(2));

    rmappdata(fig,'BridgeDragState');
    fig.Pointer = 'arrow';
    fig.WindowButtonMotionFcn = '';
    fig.WindowButtonUpFcn = '';
end

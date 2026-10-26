%% FOV
% Inputs: sensors, a cell array of structs with fields s, col and name.
% Each s contains sens_stamp, x_rel and y_rel. Optional sensor.fov rows are
% [aperture, center] in degrees. If ax exists, Refresh uses xlim(ax(1)).

fov_time_axis = [];
if exist('ax','var') && ~isempty(ax) && isgraphics(ax(1),'axes')
    fov_time_axis = ax(1);
end

fov_figure = figure('Name','FOV','NumberTitle','off','Color','w');
fov_layout = tiledlayout(fov_figure,1,2, ...
    'Padding','compact','TileSpacing','compact');
fov_layout.OuterPosition = [0 0.08 1 0.92];
fov_map_axis = nexttile(fov_layout);
fov_range_axis = nexttile(fov_layout);

fov_refresh = @(~,~) refresh_fov( ...
    fov_map_axis,fov_range_axis,sensors,fov_time_axis);
uicontrol(fov_figure,'Style','pushbutton','String','Refresh', ...
    'Units','normalized','Position',[0.01 0.01 0.1 0.05], ...
    'Callback',fov_refresh);
fov_refresh([],[]);

function refresh_fov(map_axis,range_axis,sensors,time_axis)
    t_lim = [-inf inf];
    if ~isempty(time_axis) && isgraphics(time_axis,'axes')
        t_lim = xlim(time_axis);
    end

    cla(map_axis,'reset');
    hold(map_axis,'on');
    grid(map_axis,'on');
    axis(map_axis,'equal');
    set(map_axis,'XDir','reverse');
    xlabel(map_axis,'y [m]');
    ylabel(map_axis,'x [m]');
    title(map_axis,'Sensors FOV');

    cla(range_axis,'reset');
    hold(range_axis,'on');
    grid(range_axis,'on');
    xlabel(range_axis,'range [m]');
    ylabel(range_axis,'normalized counts');
    title(range_axis,'Range distribution');

    for k = 1:numel(sensors)
        sensor = sensors{k};
        s = sensor.s;
        % Match timestamps per detection, including all matrix columns.
        % A timestamp column also supports one timestamp per frame.
        valid = isfinite(s.sens_stamp) & s.sens_stamp >= 0 & ...
            s.sens_stamp >= t_lim(1) & s.sens_stamp <= t_lim(2) & ...
            isfinite(s.x_rel) & isfinite(s.y_rel);
        x = s.x_rel(valid);
        y = s.y_rel(valid);

        plot(map_axis,y,x,'*','Color',sensor.col, ...
            'DisplayName',sensor.name);
        histogram(range_axis,hypot(x,y),'Normalization','probability', ...
            'BinWidth',1,'FaceColor',sensor.col,'DisplayName',sensor.name);

        if isfield(sensor,'fov')
            for j = 1:size(sensor.fov,1)
                angles = deg2rad(sensor.fov(j,2) + [-1 1]*sensor.fov(j,1)/2);
                plot(map_axis,[0 0; 150*sin(angles)], ...
                    [0 0; 150*cos(angles)],'-','LineWidth',0.5, ...
                    'Color',sensor.col,'HandleVisibility','off');
            end
        end
    end

    theta = linspace(0,2*pi,200);
    for radius = [25 50 75 100]
        plot(map_axis,radius*cos(theta),radius*sin(theta),'--', ...
            'LineWidth',0.3,'Color',[0.5 0.5 0.5],'HandleVisibility','off');
    end
    plot(map_axis,0,0,'d','Color','k','LineWidth',1, ...
        'MarkerSize',10,'DisplayName','Ego');
    xlim(map_axis,[-125 125]);
    xlim(range_axis,[0 150]);
    legend(map_axis,'Location','northwest');
    legend(range_axis,'Location','best');
end

%% FOV

[FoV.cam_x,FoV.cam_y]  = compute_fov(85,0);
[FoV.lid_x,FoV.lid_y]  = compute_fov(10,179);
[FoV.rad_x,FoV.rad_y]  = compute_fov(90,0);

[range.x_25m, range.y_25m]   = calculate_ellipse(25,25,0,0); 
[range.x_50m, range.y_50m]   = calculate_ellipse(50,50,0,0); 
[range.x_75m, range.y_75m]   = calculate_ellipse(75,75,0,0); 
[range.x_100m, range.y_100m] = calculate_ellipse(100,100,0,0); 

%% FIGURE
fov_fig = figure('Name','FOV', 'NumberTitle','off');
b = b + 1;
c(b) = uicontrol('Style','pushbutton', ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@fovButtonPushed);

function fovButtonPushed(~,~)
    sensors  = evalin('base','sensors');
    FoV      = evalin('base','FoV');
    range    = evalin('base','range');
    ax       = evalin('base','ax');
    col      = evalin('base','col');

    t_lim = xlim(ax(1));

    % Compute time indices per sensor
    for k = 1:numel(sensors)
        t1  = find(sensors{k}.s.sens_stamp > t_lim(1),1);
        t2  = find(sensors{k}.s.sens_stamp < t_lim(2),1,'last');
        sensors{k}.idx = t1:t2;
    end

    % ================= MAP =================
    subplot(1,2,1)
    cla reset
    ax1 = gca;
    hold on; grid on; axis equal
    title('FoV')
    xlabel('y [m]'); ylabel('x [m]')

    for k = 1:numel(sensors)
        s = sensors{k}.s;
        idx = sensors{k}.idx;
        plot(s.y_rel(idx), s.x_rel(idx), '*', ...
            'Color', sensors{k}.col, ...
            'DisplayName', sensors{k}.name);
    end

    % FOV
    plot(FoV.rad_y,  FoV.rad_x,'-','Color', col.radar,'HandleVisibility','off')
    plot(FoV.rad_y, -FoV.rad_x,'-','Color', col.radar,'HandleVisibility','off')
    plot(FoV.lid_y,  FoV.lid_x,'-','Color', col.lidar,'HandleVisibility','off')
    plot(FoV.cam_y,  FoV.cam_x,'-','Color', col.camera,'HandleVisibility','off')
    plot(FoV.cam_y, -FoV.cam_x,'-','Color', col.camera,'HandleVisibility','off')

    % Range rings
    plot(range.x_25m, range.y_25m,'--','Color',[.5 .5 .5],'HandleVisibility','off', 'LineWidth',0.8)
    plot(range.x_50m, range.y_50m,'--','Color',[.5 .5 .5],'HandleVisibility','off', 'LineWidth',0.8)
    plot(range.x_75m, range.y_75m,'--','Color',[.5 .5 .5],'HandleVisibility','off', 'LineWidth',0.8)
    plot(range.x_100m,range.y_100m,'--','Color',[.5 .5 .5],'HandleVisibility','off','LineWidth',0.8)

    plot(0,0,'d','MarkerSize',10,'DisplayName','Ego', 'Color', [0 0 0])
    legend('Location','west')

    % ================= ERROR INTENSITY =================
    subplot(1,2,2)
    cla reset
    ax2 = gca;
    hold on; grid on; axis equal
    title('Detection Error')
    xlabel('y [m]'); ylabel('x [m]')

    markers = {'o','square','^','diamond'};

    for k = 1:numel(sensors)
        s = sensors{k}.s;
        idx = sensors{k}.idx;

        err = hypot(s.x_map_err, s.y_map_err);

        scatter(s.y_rel(idx), s.x_rel(idx), [], err(idx), ...
            'filled', markers{k}, ...
            'DisplayName', sensors{k}.name);
    end

    % FOV
    plot(FoV.rad_y,  FoV.rad_x,'-','Color', col.radar,'HandleVisibility','off')
    plot(FoV.rad_y, -FoV.rad_x,'-','Color', col.radar,'HandleVisibility','off')
    plot(FoV.lid_y,  FoV.lid_x,'-','Color', col.lidar,'HandleVisibility','off')
    plot(FoV.cam_y,  FoV.cam_x,'-','Color', col.camera,'HandleVisibility','off')
    plot(FoV.cam_y, -FoV.cam_x,'-','Color', col.camera,'HandleVisibility','off')

    % Range rings
    plot(range.x_25m, range.y_25m,'--','Color',[.5 .5 .5],'HandleVisibility','off', 'LineWidth',0.8)
    plot(range.x_50m, range.y_50m,'--','Color',[.5 .5 .5],'HandleVisibility','off', 'LineWidth',0.8)
    plot(range.x_75m, range.y_75m,'--','Color',[.5 .5 .5],'HandleVisibility','off', 'LineWidth',0.8)
    plot(range.x_100m,range.y_100m,'--','Color',[.5 .5 .5],'HandleVisibility','off','LineWidth',0.8)

    plot(0,0,'d','MarkerSize',10,'DisplayName','Ego', 'Color', [0 0 0])
    legend('Location','west')

    colorbar('Location','eastoutside');
    colormap('jet');
    clim([0 3]);
    ax1Pos = get(ax1, 'Position');
    ax2Pos = get(ax2, 'Position');
    ax2Pos(3:4) = ax1Pos(3:4);
    set(ax2, 'Position', ax2Pos);
    colorbar('Position', [0.93 0.15 0.02 0.7]);  % normalized 

    linkaxes([ax1 ax2],'xy')
end


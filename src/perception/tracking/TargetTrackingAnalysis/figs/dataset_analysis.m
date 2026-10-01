figure('Name','Dataset overview', 'NumberTitle', 'off');

% Button
c = c + 1;
b(c) = uicontrol('Style','pushbutton', ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@refreshTimeButtonPushed);

function refreshTimeButtonPushed(~, ~)

    % --- fetch from base ---
    col          = evalin('base','col');
    ax           = evalin('base','ax');
    tt           = evalin('base','tt');
    sensors      = evalin('base','sensors');
    use_ref      = evalin('base','use_ref');
    use_sim_ref  = evalin('base','use_sim_ref');

    gt_on = false;
    if use_ref || use_sim_ref
        gt = evalin('base','gt');
        gt_on = true;
    end

    % --- time window ---
    t_lim = xlim(ax(1));

    [t1_tt, tend_tt] = timeWindowIdx(tt.stamp, t_lim);
    if gt_on; [t1_gt, tend_gt] = timeWindowIdx(gt.stamp, t_lim); end

    if gt_on
        tiledlayout(2,2);
    else
        tiledlayout(2,1);
    end
    nexttile; cla reset; hold on; grid on;

    % --- range ---
    if gt_on 
        data = gt.rho(t1_gt:tend_gt);
        [counts, edges] = histcounts(data);
        centers = edges(1:end-1) + diff(edges)/2;
        plot(centers, counts, 'Color', col.ref, 'DisplayName', 'gt');
    end

    % --- sensors ---
    for i = 1:numel(sensors)
        s = sensors{i}.s;
        [i1, i2] = timeWindowIdx(s.sens_stamp, t_lim);
        r = sqrt(s.x_rel(i1:i2).^2 + s.y_rel(i1:i2).^2);

        [counts, edges] = histcounts(r);
        centers = edges(1:end-1) + diff(edges)/2;

        plot(centers, counts, 'Color', sensors{i}.col, 'DisplayName', sensors{i}.name);
    end
    ylabel('count'); xlabel('range [m]'); legend show;

    % --- detection rate ---
    nexttile; cla reset; hold on; grid on;
    camera_rate = tt.buffer.cam_yolo(t1_tt:tend_tt,1);
    lidar_rate  = tt.buffer.lid_clust(t1_tt:tend_tt,1) - camera_rate;
    pp_rate     = tt.buffer.lid_pp(t1_tt:tend_tt,1) - lidar_rate - camera_rate;
    radar_rate  = tt.buffer.rad_clust(t1_tt:tend_tt,1) - pp_rate - lidar_rate - camera_rate;
    
    rates = {lidar_rate, pp_rate, radar_rate, camera_rate};
    names = ["Lidar","PP","Radar","Camera"];

    for k = 1:length(rates)
        data = rates{k};
        [counts, edges] = histcounts(data);
        centers = edges(1:end-1) + diff(edges)/2;
        plot(centers, counts, 'LineWidth', 1.5, 'Color', sensors{k}.col, 'DisplayName', names(k));
    end
    xlim([0 inf]);
    legend show
    ylabel('count');
    xlabel('detection rate [Hz]');

    if gt_on
        % --- velocity ---
        nexttile; cla reset; hold on; grid on;
        data = gt.vx(t1_gt:tend_gt);
        [counts, edges] = histcounts(data);
        centers = edges(1:end-1) + diff(edges)/2;
        plot(centers, counts, 'DisplayName', 'gt', 'Color', col.ref);
    
        ylabel('count');
        xlabel('vx [m/s]');
    
        % --- acceleration ---
        nexttile; cla reset; hold on; grid on;
        data = gt.ax(t1_gt:tend_gt);
        [counts, edges] = histcounts(data);
        centers = edges(1:end-1) + diff(edges)/2;
        plot(centers, counts, 'DisplayName', 'gt', 'Color', col.ref);
        ylabel('count');
        xlabel('ax [m/s]');
    end
end

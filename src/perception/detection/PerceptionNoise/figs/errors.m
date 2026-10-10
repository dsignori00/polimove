
%% SERIES 

error_fig = figure('Name','Errors','NumberTitle','off');
error_tabs = uitabgroup(error_fig);

% ---------------- MAP ----------------
map_tab = uitab(error_tabs,'Title','Map');
map_layout = tiledlayout(map_tab,3,1,'Padding','compact','TileSpacing','compact');
map_fields = {'x_map_err','y_map_err', 'yaw_map_err'};
map_label = {'x [m]','y [m]', 'yaw [deg]'};

for j = 1:numel(map_fields)
    ax(f) = nexttile(map_layout); hold on; grid on; f=f+1; %#ok<SAGROW>
    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')
    for i = 1:numel(sensors)
        plot_detections(sensors{i}.s.sens_stamp, ...
            gated_error(sensors{i}.s,map_fields{j},err_thr), ...
            1,sensors{i}.col,sensors{i}.name)
    end
    xlim([0 inf]);
    ylabel(map_label{j});
    legend show
end
xlabel('timestamp [s]');

% ---------------- COG ----------------
cog_tab = uitab(error_tabs,'Title','CoG');
cog_layout = tiledlayout(cog_tab,3,1,'Padding','compact','TileSpacing','compact');
cog_fields = {'x_rel_err','y_rel_err','yaw_rel_err'};
cog_label = {'x [m]','y [m]', 'yaw [deg]'};

for j = 1:numel(cog_fields)
    ax(f) = nexttile(cog_layout); hold on; grid on; f=f+1;
    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')
    for i = 1:numel(sensors)
        plot_detections(sensors{i}.s.sens_stamp, ...
            gated_error(sensors{i}.s,cog_fields{j},err_thr), ...
            1,sensors{i}.col,sensors{i}.name)
    end
    xlim([0 inf]);
    ylabel(cog_label{j});
    legend show
end
xlabel('timestamp [s]');

% ---------------- RANGE ----------------
has_range_rate = any(cellfun(@(sensor) sensor.has_rho_dot,sensors));
range_tab = uitab(error_tabs,'Title','Range');
range_layout = tiledlayout(range_tab,1 + double(has_range_rate),1, ...
    'Padding','compact','TileSpacing','compact');

ax(f) = nexttile(range_layout); hold on; grid on; f=f+1;
yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')
for i = 1:numel(sensors)
    s = sensors{i}.s;
    range_err = hypot(s.x_rel,s.y_rel) - hypot(s.x_rel_gt,s.y_rel_gt);
    range_err(~position_gate(s,err_thr)) = NaN;

    plot_detections(s.sens_stamp,range_err,1,sensors{i}.col,sensors{i}.name)
end
xlim([0 inf]); ylim(y_err_lim); ylabel('error [m]');
title('range'); legend show

if has_range_rate
    ax(f) = nexttile(range_layout); hold on; grid on; f=f+1;
    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')
    for i = 1:numel(sensors)
        if sensors{i}.has_rho_dot
            s = sensors{i}.s;
            rho_dot_err = s.rho_dot_err;
            rho_dot_err(~position_gate(s,err_thr)) = NaN;
            plot_detections(s.sens_stamp,rho_dot_err,1, ...
                sensors{i}.col,sensors{i}.name)
        end
    end
    xlim([0 inf]); ylim(y_err_lim); ylabel('error [m/s]');
    title('range rate'); legend show
end
xlabel('timestamp [s]');

function err = gated_error(sensor, field, err_thr)
    gate = position_gate(sensor,err_thr);
    err = sensor.(field)(:,1);
    err(~gate) = NaN;
end

function gate = position_gate(sensor,err_thr)
    gate = hypot(sensor.x_map_err(:,1),sensor.y_map_err(:,1)) < err_thr;
end

%% SUMMARY 

error_summary_tab = uitab(error_tabs,'Title','Summary');

% Button
b = b + 1;
c(b) = uicontrol('Style','pushbutton', ...
    'Parent',error_summary_tab, ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@refreshErrorSummaryButtonPushed);

function refreshErrorSummaryButtonPushed(~,~)

    % --- fetch from base ---
    sensors  = evalin('base','sensors');
    ax_ref   = evalin('base','ax');
    err_thr  = evalin('base','err_thr');
    summary_tab = evalin('base','error_summary_tab');

    % --- compute stats (PER SENSOR) ---
    for k = 1:numel(sensors)
        s = sensors{k}.s;

        % time window from reference axis
        t_lim = xlim(ax_ref(1));
        [t1, tend] = timeWindowIdx(s.sens_stamp, t_lim);

        time = false(size(s.sens_stamp));
        time(t1:tend) = true;

        ass = hypot(s.x_map_err, s.y_map_err) < err_thr;
        time_mat = repmat(time, 1, size(s.x_map_err,2));
        idx = ass & time_mat;
        s.idx = idx;

        stats = {'x_rel','y_rel','yaw_map'};
        for l = stats
            err = s.([l{1} '_err']);
            s.([l{1} '_std'])  = std(err(idx));
            s.([l{1} '_mean']) = mean(err(idx));
        end

        if sensors{k}.has_rho_dot
            s.rho_dot_std  = std(s.rho_dot_err(idx));
            s.rho_dot_mean = mean(s.rho_dot_err(idx));
        end

        [s.x_ellipse, s.y_ellipse] = calculate_ellipse( ...
            s.x_rel_std, s.y_rel_std, ...
            s.x_rel_mean, s.y_rel_mean);

        sensors{k}.s = s;
    end

    % --- Positional errors (ellipse) ---
    ax_pos = subplot(2,2,[1 3],'Parent',summary_tab);
    cla reset; hold(ax_pos,'on'); grid(ax_pos,'on'); axis(ax_pos,'equal')

    for i = 1:numel(sensors)
        si = sensors{i}.s;

        plot(ax_pos, si.y_ellipse, si.x_ellipse, ...
            'Color', sensors{i}.col, ...
            'LineWidth', 2, ...
            'DisplayName', sensors{i}.name)

        scatter(ax_pos, si.y_rel_mean, si.x_rel_mean, ...
            'o', ...
            'MarkerFaceColor', sensors{i}.col, ...
            'MarkerEdgeColor', sensors{i}.col, ...
            'HandleVisibility','off')

        str = sprintf('σ_x = %.2f m\nσ_y = %.2f m', ...
            sensors{i}.s.x_rel_std, sensors{i}.s.y_rel_std);

        text(ax_pos, si.y_rel_mean, si.x_rel_mean, str, ...
            'Interpreter','none', ...
            'VerticalAlignment','bottom', ...
            'HorizontalAlignment','right', ...
            'FontSize',12, ...
            'FontWeight','bold', ...
            'Color','k', ...
            'BackgroundColor',[1 1 1 0.75], ...
            'EdgeColor', sensors{i}.col, ...
            'Margin',3, ...
            'HandleVisibility','off');
    end

    xline(ax_pos,0,'--','LineWidth',0.3,'HandleVisibility','off')
    yline(ax_pos,0,'--','LineWidth',0.3,'HandleVisibility','off')
    title(ax_pos,'Positional Errors')
    xlabel(ax_pos,'y rel [m]')
    ylabel(ax_pos,'x rel [m]')
    legend(ax_pos,'show')

    % --- Range-rate error (Radar only) ---
    rho_dot_idx = find(cellfun(@(s) s.has_rho_dot, sensors),1);

    if ~isempty(rho_dot_idx)
        ax_rho = subplot(2,2,2,'Parent',summary_tab);
        cla reset; hold(ax_rho,'on'); grid(ax_rho,'on')

        si = sensors{rho_dot_idx}.s;

        boxplot(ax_rho, si.rho_dot_err(si.idx), 'Symbol','')
        for k = -1:2:1
        yline(ax_rho,  k*si.rho_dot_std,  '--', ...
            string(k) + 'σ =' + string(si.rho_dot_std), ...
            'HandleVisibility','off', ...
            'LineWidth', 1, ...
            'Interpreter','none');
        end
        ylim(ax_rho, [-2*si.rho_dot_std  2*si.rho_dot_std])
        xlim(ax_rho, [0.8 1.2])
        xticklabels(ax_rho,[])

        title(ax_rho,'Range Rate Error')
        ylabel(ax_rho,'[m/s]')
        xlabel(ax_rho, sensors{rho_dot_idx}.name)
    end

    % --- Heading error ---
    ax_yaw = subplot(2,2,4,'Parent',summary_tab);
    cla(ax_yaw); hold(ax_yaw,'on'); grid(ax_yaw,'on')

    yaw_err   = [];
    group     = {};
    YawMaxStd = 0;

    for i = 1:numel(sensors)
        si = sensors{i}.s;

        e = si.yaw_map_err(si.idx);
        yaw_err = [yaw_err; e];
        group   = [group; repmat({sensors{i}.name}, numel(e), 1)];

        YawMaxStd = max(YawMaxStd, si.yaw_map_std);
    end

    boxplot(ax_yaw, yaw_err, group, 'Symbol','')
    yline(ax_yaw,0,'--','LineWidth',0.3,'HandleVisibility','off')
    ylim(ax_yaw, [-2*YawMaxStd  2*YawMaxStd])

    title(ax_yaw,'Heading Error')
    ylabel(ax_yaw,'[deg]')

end


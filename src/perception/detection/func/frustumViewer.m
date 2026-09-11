function fig = frustumViewer(streams, overlays, cfg)

    timeline = streams(1).t;

    n_samples = numel(timeline);
    selected_samples = 1:n_samples;
    selected_idx = 1;

    fig = figure( ...
        'Name', 'Frustum viewer', ...
        'NumberTitle', 'off');

    ax = axes(fig);

    uicontrol(fig, ...
        'Style', 'pushbutton', ...
        'String', 'Refresh', ...
        'Units', 'normalized', ...
        'Position', [0.01 0.01 0.1 0.05], ...
        'Callback', @refreshTimeButtonPushed);

    hold(ax, 'on');
    grid(ax, 'on');
    box(ax, 'on');
    axis(ax, 'equal');

    xlim(ax, cfg.x_lim);
    ylim(ax, cfg.y_lim);

    xlabel(ax, 'x [m]');
    ylabel(ax, 'y [m]');

    cloud_h = gobjects(numel(streams), 1);

    for i = 1:numel(streams)
        cloud_h(i) = scatter( ...
            ax, nan, nan, ...
            cfg.point_size, ...
            streams(i).color, ...
            'filled', ...
            'DisplayName', char(streams(i).name));
    end

    overlay_h = gobjects(0);

    fig.WindowKeyPressFcn = @onKeyPress;

    updatePlot();

    legend(ax, 'Location', 'best');


    function refreshTimeButtonPushed(~, ~)

        if ~isfield(cfg, 'time_axis') || ...
                ~isgraphics(cfg.time_axis, 'axes')
            warning('FrustumViewer:MissingTimeAxis', ...
                'No valid time axis is available for selecting the time range.');
            return
        end

        t_lim = xlim(cfg.time_axis);
        selected_samples = find( ...
            isfinite(timeline) & ...
            timeline >= t_lim(1) & ...
            timeline <= t_lim(2));
        selected_idx = 1;

        if isempty(selected_samples)
            clearPlot(t_lim);
            return
        end

        updatePlot();
    end


    function onKeyPress(~, event)

        if isempty(selected_samples)
            return
        end

        switch event.Key

            case 'rightarrow'
                selected_idx = selected_idx + 1;

            case 'leftarrow'
                selected_idx = selected_idx - 1;

            case 'uparrow'
                selected_idx = selected_idx + cfg.jump;

            case 'downarrow'
                selected_idx = selected_idx - cfg.jump;

            case 'home'
                selected_idx = 1;

            case 'end'
                selected_idx = numel(selected_samples);

            otherwise
                return
        end

        selected_idx = max(1, min(numel(selected_samples), selected_idx));

        updatePlot();
    end


    function updatePlot()

        sample_idx = selected_samples(selected_idx);
        current_t = timeline(sample_idx);

        for i = 1:numel(streams)

            [idx, dt] = nearestSample(streams(i).t, current_t);

            if dt > cfg.max_dt
                set(cloud_h(i), 'XData', nan, 'YData', nan);
                continue
            end

            points = streams(i).points{idx};

            set(cloud_h(i), ...
                'XData', points(:,1), ...
                'YData', points(:,2));
        end


        if ~isempty(overlay_h)
            delete(overlay_h(isgraphics(overlay_h)));
        end

        overlay_h = gobjects(0);

        for i = 1:numel(overlays)

            overlay = overlays{i};

            [idx, dt] = nearestSample(overlay.t, current_t);

            if dt <= overlay.max_dt
                h = overlay.draw(ax, idx);
                overlay_h = [overlay_h; h(:)]; %#ok<AGROW>
            end
        end


        title(ax, sprintf( ...
            't: %.3f s (%d / %d)', ...
            current_t, selected_idx, numel(selected_samples)));

        drawnow limitrate
    end


    function clearPlot(t_lim)

        for i = 1:numel(cloud_h)
            set(cloud_h(i), 'XData', nan, 'YData', nan);
        end

        if ~isempty(overlay_h)
            delete(overlay_h(isgraphics(overlay_h)));
        end
        overlay_h = gobjects(0);

        title(ax, sprintf( ...
            'No samples in selected range [%.3f, %.3f] s', ...
            t_lim(1), t_lim(2)));

        drawnow
    end
end


function [idx, dt] = nearestSample(t, target_t)

    [dt, idx] = min(abs(double(t(:)) - target_t), [], 'omitnan');
end
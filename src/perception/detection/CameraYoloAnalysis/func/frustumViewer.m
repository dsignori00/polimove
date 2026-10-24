function fig = frustumViewer(streams, overlays, cfg)

    %% Reference timeline

    stream_names = string({streams.name});

    reference_idx = find( ...
        stream_names == string(cfg.reference_stream), ...
        1, 'first');

    if isempty(reference_idx)
        error( ...
            'FrustumViewer:MissingReferenceStream', ...
            'Reference stream "%s" was not found. Available streams: %s', ...
            cfg.reference_stream, ...
            strjoin(stream_names, ', '));
    end

    % The whole viewer uses ONLY this stream as its timeline.
    timeline = double(streams(reference_idx).t(:));

    % Only valid timestamps from the reference stream are navigable.
    selected_samples = find(isfinite(timeline));

    if isempty(selected_samples)
        error( ...
            'FrustumViewer:EmptyReferenceTimeline', ...
            'Reference stream "%s" contains no valid timestamps.', ...
            cfg.reference_stream);
    end

    selected_idx = 1;


    %% Figure

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


    %% Point-cloud handles

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


    %% Refresh selected time window

    function refreshTimeButtonPushed(~, ~)

        if ~isfield(cfg, 'time_axis') || ...
                ~isgraphics(cfg.time_axis, 'axes')

            warning( ...
                'FrustumViewer:MissingTimeAxis', ...
                'No valid time axis is available for selecting the time range.');

            return
        end

        t_lim = xlim(cfg.time_axis);

        % IMPORTANT:
        % selected samples come ONLY from the reference timeline.
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


    %% Keyboard navigation

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

        selected_idx = max( ...
            1, ...
            min(numel(selected_samples), selected_idx));

        updatePlot();
    end


    %% Update visualization

    function updatePlot()

        % Index into the reference stream.
        sample_idx = selected_samples(selected_idx);

        % This is THE timestamp used by the entire viewer.
        current_t = timeline(sample_idx);


        %% Point-cloud streams

        for i = 1:numel(streams)

            if i == reference_idx

                % Reference stream:
                % use the exact sample that generated current_t.
                idx = sample_idx;
                dt = 0;

            else

                % Every other stream is synchronized against
                % the reference timestamp.
                [idx, dt] = nearestSample( ...
                    streams(i).t, ...
                    current_t);
            end


            if isempty(idx) || ~isfinite(dt) || dt > cfg.max_dt

                set( ...
                    cloud_h(i), ...
                    'XData', nan, ...
                    'YData', nan);

                continue
            end


            points = streams(i).points{idx};

            if isempty(points)

                set( ...
                    cloud_h(i), ...
                    'XData', nan, ...
                    'YData', nan);

                continue
            end

            set( ...
                cloud_h(i), ...
                'XData', points(:,1), ...
                'YData', points(:,2));
        end


        %% Clear old overlays

        if ~isempty(overlay_h)
            delete(overlay_h(isgraphics(overlay_h)));
        end

        overlay_h = gobjects(0);


        %% Overlays

        for i = 1:numel(overlays)

            overlay = overlays{i};

            [idx, dt] = nearestSample( ...
                overlay.t, ...
                current_t);

            if ~isempty(idx) && ...
                    isfinite(dt) && ...
                    dt <= overlay.max_dt

                h = overlay.draw(ax, idx);

                overlay_h = [ ...
                    overlay_h; ...
                    h(:) ...
                ]; %#ok<AGROW>
            end
        end


        %% Title

        title(ax, sprintf( ...
            '%s reference | t: %.3f s (%d / %d)', ...
            cfg.reference_stream, ...
            current_t, ...
            selected_idx, ...
            numel(selected_samples)));

        drawnow limitrate
    end


    %% Clear plot

    function clearPlot(t_lim)

        for i = 1:numel(cloud_h)

            set( ...
                cloud_h(i), ...
                'XData', nan, ...
                'YData', nan);
        end

        if ~isempty(overlay_h)
            delete(overlay_h(isgraphics(overlay_h)));
        end

        overlay_h = gobjects(0);

        title(ax, sprintf( ...
            'No %s samples in selected range [%.3f, %.3f] s', ...
            cfg.reference_stream, ...
            t_lim(1), ...
            t_lim(2)));

        drawnow
    end
end


function [idx, dt] = nearestSample(t, target_t)

    t = double(t(:));

    valid = isfinite(t);

    if ~any(valid) || ~isfinite(target_t)
        idx = [];
        dt = inf;
        return
    end

    valid_idx = find(valid);

    [dt, local_idx] = min( ...
        abs(t(valid) - target_t));

    idx = valid_idx(local_idx);
end
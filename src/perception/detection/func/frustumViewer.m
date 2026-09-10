function fig = frustumViewer(streams, overlays, cfg)

    timeline = streams(1).t;

    n_samples = numel(timeline);
    sample_idx = 1;

    fig = figure( ...
        'Name', 'Frustum viewer', ...
        'NumberTitle', 'off');

    ax = axes(fig);

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


    function onKeyPress(~, event)

        switch event.Key

            case 'rightarrow'
                sample_idx = sample_idx + 1;

            case 'leftarrow'
                sample_idx = sample_idx - 1;

            case 'uparrow'
                sample_idx = sample_idx + cfg.jump;

            case 'downarrow'
                sample_idx = sample_idx - cfg.jump;

            case 'home'
                sample_idx = 1;

            case 'end'
                sample_idx = n_samples;

            otherwise
                return
        end

        sample_idx = max(1, min(n_samples, sample_idx));

        updatePlot();
    end


    function updatePlot()

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
            'Sample %d / %d | t = %.3f s', ...
            sample_idx, n_samples, current_t));

        drawnow limitrate
    end
end


function [idx, dt] = nearestSample(t, target_t)

    [dt, idx] = min(abs(double(t(:)) - target_t), [], 'omitnan');
end
%% FRUSTUM VISUALIZATION
assert(sum(VISUALIZE_FRUSTUM)<=1,"VISUALIZE_FRUSTUM must have at most one camera selected")

camera_idx = find(VISUALIZE_FRUSTUM == true, 1, 'first');
camera_name = CAMERA_NAMES(camera_idx);

streams = loadFrustumStreams(log, camera_name);

cfg.x_lim = [-150 150];
cfg.y_lim = [-45 45];
cfg.point_size = 7;
cfg.max_dt = 0.10;
cfg.jump = 10;

% Use the same selected time window as the map visualization.
time_axes = ax(isgraphics(ax, 'axes'));
if ~isempty(time_axes)
    cfg.time_axis = time_axes(1);
end

overlays = {};

frustumViewer(streams, overlays, cfg);
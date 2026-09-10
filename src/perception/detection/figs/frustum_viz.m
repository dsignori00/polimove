%% FRUSTUM VISUALIZATION

camera_idx = 2;
camera_name = "camera_fr";

streams = loadFrustumStreams(log, camera_name);

cfg.x_lim = [-150 150];
cfg.y_lim = [-45 45];
cfg.point_size = 7;
cfg.max_dt = 0.10;
cfg.jump = 10;

overlays = {};

frustumViewer(streams, overlays, cfg);
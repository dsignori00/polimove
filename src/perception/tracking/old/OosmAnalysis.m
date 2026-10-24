clearvars -except log;
clc;

% load log
if (~exist('log', 'var'))
    [file, path] = uigetfile('*.mat', 'Load log');
    load(fullfile(path, file), 'log');
end

% style
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultTextInterpreter', 'none');
set(0, 'DefaultLegendInterpreter', 'none');
set(0, 'DefaultLineLineWidth', 1);

lidar.t = log.perception__lidar_cluster__bag_timestamp;
lidar.stamp = log.perception__lidar_cluster__sensor_stamp__tot;
lidar.count = log.perception__lidar_cluster__count;
camera.t = log.perception__camera_yolo__bag_timestamp;
camera.stamp = log.perception__camera_yolo__sensor_stamp__tot;
camera.count = log.perception__camera_yolo__count;
pp.t = log.perception__lidar_pp__bag_timestamp;
pp.stamp = log.perception__lidar_pp__sensor_stamp__tot;
pp.count = log.perception__lidar_pp__count;
radar.t = log.perception__radar_cluster__bag_timestamp;
radar.stamp = log.perception__radar_cluster__sensor_stamp__tot;
radar.count = log.perception__radar_cluster__count;

%% Select time interval

t_min = 257;
t_max = 258.5;

meas_index = 1;
idx_lidar = find(lidar.t >= t_min & lidar.t < t_max & lidar.count > 0);
idx_radar = find(radar.t >= t_min & radar.t < t_max & radar.count > 0);
idx_pp = find(pp.t >= t_min & pp.t < t_max & pp.count > 0);
idx_camera = find(camera.t >= t_min & camera.t < t_max & camera.count > 0);
num_meas = length(idx_lidar) + length(idx_radar) + length(idx_pp) + length(idx_camera);

for i = 1:length(idx_lidar)
    idx = idx_lidar(i);
    meas(meas_index).t = lidar.t(idx); %#ok<SAGROW>
    meas(meas_index).stamp = lidar.stamp(idx); %#ok<SAGROW>
    meas(meas_index).source = 1; %#ok<SAGROW>

    meas_index = meas_index + 1;
end

for i = 1:length(idx_radar)
    idx = idx_radar(i);
    meas(meas_index).t = radar.t(idx);
    meas(meas_index).stamp = radar.stamp(idx);
    meas(meas_index).source = 2;

    meas_index = meas_index + 1;
end

for i = 1:length(idx_pp)
    idx = idx_pp(i);
    meas(meas_index).t = pp.t(idx);
    meas(meas_index).stamp = pp.stamp(idx);
    meas(meas_index).source = 3;

    meas_index = meas_index + 1;
end

for i = 1:length(idx_camera)
    idx = idx_camera(i);
    meas(meas_index).t = camera.t(idx);
    meas(meas_index).stamp = camera.stamp(idx);
    meas(meas_index).source = 4;

    meas_index = meas_index + 1;
end

meas_t_array = zeros(size(length(meas)));

for i = 1:length(meas)
    meas_t_array(i) = meas(i).t;
end

[~, sorted_idx] = sort(meas_t_array);
meas = meas(sorted_idx);

meas(1).diff = 0;

for i = 2:length(meas)
    meas(i).diff = meas(i).stamp - meas(i - 1).stamp;
end

clear meas_t_array
meas_t_array = zeros(size(length(meas)));
meas_diff_array = zeros(size(length(meas)));
meas_source_array = zeros(size(length(meas)));
meas_stamp_array = zeros(size(length(meas)));

for i = 1:length(meas)
    meas_t_array(i) = meas(i).t;
    meas_diff_array(i) = meas(i).diff;
    meas_source_array(i) = meas(i).source;
    meas_stamp_array(i) = meas(i).stamp;
end

idx_oosm = find(meas_diff_array < 0);
oosm = meas(idx_oosm);

oosm_t_array = zeros(size(length(meas)));
oosm_diff_array = zeros(size(length(meas)));
oosm_source_array = zeros(size(length(meas)));
oosm_stamp_array = zeros(size(length(meas)));

for i = 1:length(oosm)
    oosm_t_array(i) = oosm(i).t;
    oosm_diff_array(i) = oosm(i).diff;
    oosm_source_array(i) = oosm(i).source;
    oosm_stamp_array(i) = oosm(i).stamp;
end

oosm_num = length(oosm);
meas_num = length(meas);

%% oosm count

figure("Name", "OOSM")

plot(meas_t_array - meas(1).t, meas_diff_array, 'Color', [0 0 0])
title("OOSM - Count: ")
xlabel("Time [s]")
ylabel("Time difference [s]")
ylim([-0.15 0.17])
hold on
scatter(oosm_t_array - meas(1).t, oosm_diff_array, 50, 'Marker', 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 2)
yline(0.0, '--');
legend('Measurements', 'OOSM')

%% oosm input

lidar_in = meas(meas_source_array == 1);
radar_in = meas(meas_source_array == 2);
pp_in = meas(meas_source_array == 3);
camera_in = meas(meas_source_array == 4);

lidar_t_array = zeros(size(length(meas)));
lidar_diff_array = zeros(size(length(meas)));
lidar_stamp_array = zeros(size(length(meas)));

for i = 1:length(lidar_in)
    lidar_t_array(i) = lidar_in(i).t;
    lidar_diff_array(i) = lidar_in(i).diff;
    lidar_stamp_array(i) = lidar_in(i).stamp;
end

radar_t_array = zeros(size(length(meas)));
radar_diff_array = zeros(size(length(meas)));
radar_stamp_array = zeros(size(length(meas)));

for i = 1:length(radar_in)
    radar_t_array(i) = radar_in(i).t;
    radar_diff_array(i) = radar_in(i).diff;
    radar_stamp_array(i) = radar_in(i).stamp;
end

pp_t_array = zeros(size(length(meas)));
pp_diff_array = zeros(size(length(meas)));
pp_stamp_array = zeros(size(length(meas)));

for i = 1:length(pp_in)
    pp_t_array(i) = pp_in(i).t;
    pp_diff_array(i) = pp_in(i).diff;
    pp_stamp_array(i) = pp_in(i).stamp;
end

camera_t_array = zeros(size(length(meas)));
camera_diff_array = zeros(size(length(meas)));
camera_stamp_array = zeros(size(length(meas)));

for i = 1:length(camera_in)
    camera_t_array(i) = camera_in(i).t;
    camera_diff_array(i) = camera_in(i).diff;
    camera_stamp_array(i) = camera_in(i).stamp;
end

verde_scuro = [77, 155, 62] / 255;

figure("Name", "OOSM input")

lidar_plot = scatter(lidar_t_array - meas(1).t, lidar_diff_array, 50, 'blue', 'filled', 'LineWidth', 2);
title("OOSM - Input Source ")
xlabel("Time [s]")
ylabel("Time difference [s]")
hold on
radar_plot = scatter(radar_t_array - meas(1).t, radar_diff_array, 50, 'black', 'filled', 'LineWidth', 2);
pp_plot = scatter(pp_t_array - meas(1).t, pp_diff_array, 50, verde_scuro, 'filled', 'LineWidth', 2);
camera_plot = scatter(camera_t_array - meas(1).t, camera_diff_array, 50, 'red', 'filled', 'LineWidth', 2);
ylim([-0.15, 0.2])
sfondo = plot(meas_t_array - meas(1).t, meas_diff_array, 'Color', [0, 0, 0]);
uistack(sfondo, 'bottom');
yline(0.0, '--');
legend([lidar_plot, radar_plot, pp_plot, camera_plot], 'LiDAR Clustering', 'RaDAR Clustering', 'LiDAR Pointpillars', 'Camera');

%% Reprocessing Analyses

line2 = lidar_t_array(14) - meas(2).stamp;
line1 = lidar_stamp_array(14) - meas(2).stamp;
lidar_t_array(14) = [];
lidar_stamp_array(14) = [];
xx = line1:0.01:line2;
yy = line1 * ones(size(xx));
figure("Name", "Reprocessing Analyses")
title("OOSM - Reprocessing")
xlabel("Time [s]")
ylabel("Sensor Timestamp [s]")
hold on
fill([line1, line2, line2, line1], [line1, line1, 1.6, 1.6], [1, 1, 0], ...
    'FaceAlpha', 0.3, 'EdgeColor', 'none');
radar_plot = scatter(radar_t_array - meas(2).stamp, radar_stamp_array - meas(2).stamp, 30, 'black', 'filled', 'LineWidth', 2);
pp_plot = scatter(pp_t_array - meas(2).stamp, pp_stamp_array - meas(2).stamp, 30, verde_scuro, 'filled', 'LineWidth', 2);
camera_plot = scatter(camera_t_array - meas(2).stamp, camera_stamp_array - meas(2).stamp, 30, 'red', 'filled', 'LineWidth', 2);
lidar_plot = scatter(lidar_t_array - meas(2).stamp, lidar_stamp_array - meas(2).stamp, 30, 'blue', 'filled', 'LineWidth', 2);
scatter(line2, line1, 50, 'blue', 'x', 'LineWidth', 2);
% plot(xx,yy,'Color','k','LineStyle','--')
% xline(line1,'--')
% xline(line2,'--')
grid on
legend([lidar_plot, radar_plot, pp_plot, camera_plot], 'LiDAR Clustering', 'RaDAR Clustering', 'LiDAR Pointpillars', 'Camera', Location = 'northwest')

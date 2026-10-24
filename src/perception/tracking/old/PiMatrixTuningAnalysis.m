close all
clearvars -except log

use_ref = true;

% load log
if (~exist('log', 'var'))
    [file, path] = uigetfile('*.mat', 'Load log');
    load(fullfile(path, file), 'log');
end

DateTime = datetime(log.time_offset_nsec, 'ConvertFrom', 'epochtime', 'TicksPerSecond', 1e9, 'Format', 'dd-MMM-yyyy HH:mm:ss');
f = 1;

% style
set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultTextInterpreter', 'none');
set(0, 'DefaultLegendInterpreter', 'none');
set(0, 'DefaultLineLineWidth', 2);

%% Load data

% TARGET TRACKING MAIN
tt_stamp = log.perception__opp__stamp__tot;
% relative
tt_x_rel = log.perception__opp__opponents__x_rel;
tt_y_rel = log.perception__opp__opponents__y_rel;
tt_x_rel(tt_x_rel == 0) = nan;
tt_y_rel(tt_y_rel == 0) = nan;
tt_rho_dot = log.perception__opp__opponents__rho_dot;
tt_rho_dot(tt_rho_dot == 0) = nan;
tt_yaw_rel = log.perception__opp__opponents__psi_rel;
tt_yaw_rel(tt_yaw_rel == 0) = nan;
% map
tt_x_map = log.perception__opp__opponents__x_geom;
tt_y_map = log.perception__opp__opponents__y_geom;
tt_x_map(tt_x_map == 0) = nan;
tt_y_map(tt_y_map == 0) = nan;
tt_vx = log.perception__opp__opponents__vx;
tt_vx(tt_vx == 0) = nan;
tt_ax = log.perception__opp__opponents__ax;
tt_ax(tt_ax == 0) = nan;
tt_yaw_map = log.perception__opp__opponents__psi;
tt_yaw_map(tt_yaw_map == 0) = nan;

% TARGET TRACKING REF
if (use_ref)
    tt_stamp_ref = log.sim_out__stamp__tot;
    % map
    tt_x_map_ref = log.sim_out__opp_x;
    tt_y_map_ref = log.sim_out__opp_y;
    tt_x_map_ref(tt_x_map_ref == 0) = nan;
    tt_y_map_ref(tt_y_map_ref == 0) = nan;
    tt_vx_ref = log.sim_out__opp_vx;
    tt_vx_ref(tt_vx_ref == 0) = nan;
    tt_ax_ref = log.sim_out__opp_ax;
    tt_ax_ref(tt_ax_ref == 0) = nan;
    tt_yaw_map_ref = log.sim_out__opp_psi;
    tt_yaw_map_ref(tt_yaw_map_ref == 0) = nan;
end

col.tt = '#0072BD';
col.lidar = '#77AC30';
col.radar = '#4DBEEE';
col.camera = '#EDB120';
col.pp = '#D95319';
col.err = '#77AC30';
col.ref = '#FF00FF';

%% Interpolation
% The reference values are interpolated on the data timestamp

for i = 1:length(tt_stamp_ref)
    tt_x_map_ref_inter = interp1(tt_stamp_ref, tt_x_map_ref, tt_stamp);
    tt_y_map_ref_inter = interp1(tt_stamp_ref, tt_y_map_ref, tt_stamp);
    tt_vx_ref_inter = interp1(tt_stamp_ref, tt_vx_ref, tt_stamp);
    tt_ax_ref_inter = interp1(tt_stamp_ref, tt_ax_ref, tt_stamp);
    tt_yaw_map_ref_inter = interp1(tt_stamp_ref, tt_yaw_map_ref, tt_stamp);
end

%% Plot
figure('name', 'State Map');
tiledlayout(3, 2, 'Padding', 'compact');

% x
axes(f) = nexttile([1, 1]);
f = f + 1;
hold on;
plot(tt_stamp, tt_x_map, 'Color', col.tt);

if (use_ref)
    plot(tt_stamp_ref, tt_x_map_ref, 'Color', col.ref);
end

legend('track', 'ref');
grid on;
title('x map [m]');

%y
axes(f) = nexttile([1, 1]);
f = f + 1;
hold on;
plot(tt_stamp, tt_y_map, 'Color', col.tt);

if (use_ref)
    plot(tt_stamp_ref, tt_y_map_ref, 'Color', col.ref);
end

grid on;
title('y map [m]');
legend('track', 'ref');

% vx
axes(f) = nexttile([1, 1]);
f = f + 1;
hold on;
plot(tt_stamp, tt_vx, 'Color', col.tt);

if (use_ref)
    plot(tt_stamp_ref, tt_vx_ref, 'Color', col.ref);
end

grid on;
title('vx [m/s]');
legend('track', 'ref');

% ax
axes(f) = nexttile([1, 1]);
f = f + 1;
hold on;
plot(tt_stamp, tt_ax, 'Color', col.tt);

if (use_ref)
    plot(tt_stamp_ref, tt_ax_ref, 'Color', col.ref);
end

grid on;
title('ax [m/s^2]');
legend('track', 'ref');

% yaw
tt_yaw_map_ref_inter = unwrap(tt_yaw_map_ref_inter);

for i = 1:length(tt_yaw_map)
    tt_yaw_map(i) = unwrapAngleSmart(tt_yaw_map(i), tt_yaw_map_ref_inter(i));
end

axes(f) = nexttile([1, 2]);
f = f + 1;
hold on;
plot(tt_stamp, rad2deg(tt_yaw_map), 'Color', col.tt);

if (use_ref)
    plot(tt_stamp, rad2deg(tt_yaw_map_ref_inter), 'Color', col.ref);
end

grid on;
title('yaw [deg]');
legend('track', 'ref');

%% Compute TT Error

tt_x_map_error = tt_x_map - tt_x_map_ref_inter;
tt_y_map_error = tt_y_map - tt_y_map_ref_inter;
tt_vx_error = tt_vx - tt_vx_ref_inter;
tt_ax_error = tt_ax - tt_ax_ref_inter;
tt_yaw_error = tt_yaw_map - tt_yaw_map_ref_inter;

tt_x_map_error = tt_x_map_error(~isnan(tt_x_map_error));
tt_y_map_error = tt_y_map_error(~isnan(tt_y_map_error));
tt_vx_error = tt_vx_error(~isnan(tt_vx_error));
tt_ax_error = tt_ax_error(~isnan(tt_ax_error));
tt_yaw_error = tt_yaw_error(~isnan(tt_yaw_error));

tt_error_std = [std(tt_x_map_error); std(tt_y_map_error); std(tt_vx_error); std(tt_ax_error); std(rad2deg(tt_yaw_error))];

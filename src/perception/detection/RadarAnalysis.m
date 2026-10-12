close all
clearvars -except log log2 trajDatabase gt ego ego2 rad1 rad2 track

ground_truth = true;
compare      = true;

BRIDGE_POSITION.THRESHOLD = 3;
TEMPORAL_GATE = 0.1; % [s]
SPATIAL_GATE = 2.0;  % [m]
STATISTICS_BOX_POSITION = [0.14 0.78 0.32 0.16];

NAME_1 = "Radar";
NAME_2 = "Radar - 2";

%#ok<*UNRCH>
%#ok<*INUSD>

%% Paths

proj = currentProject();
addpath(fullfile(proj.RootFolder,'src','perception','utils'));

%% Load Data

%load database
if(~exist('trajDatabase','var'))
    trajDatabase = choose_database();
    if(isempty(trajDatabase))
        error('No database selected');
    else
        load(trajDatabase);
    end
end

% load log
if (~exist('log','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    load(fullfile(path,file));
end

% load log2
if (compare && ~exist('log2','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log 2');
    tmp = load(fullfile(path,file));
    log2 = tmp.log;
end

if(ground_truth)
    if (~exist('gt','var'))
        [file,path] = uigetfile(fullfile(get_gt_dir(),'*.mat'),'Load ground truth log');
        tmp = load(fullfile(path,file));
        gt = tmp.out;
    end
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% PLOT DATA

% style
set(0,'DefaultFigureWindowStyle','docked');
set(0,'DefaultTextInterpreter', 'none');
set(0,'DefaultLegendInterpreter', 'none');
set(0, 'DefaultLineLineWidth', 2);
x_lim = [0 inf];

%% NAMING
col.radar = '#0072BD';
col.radar2 = '#FF0000';
col.ref = '#000000';
sz=3; % Marker size
f=1;

% RADAR CLUSTERING DETECTIONS
rad1 = get_radar_fields(log);
if(compare); rad2 = get_radar_fields(log2); end

if ~exist('track', 'var')
    track = -1;
end
tracks = [0,1,2,3];
while ~ismember(track, tracks)
    disp( "Choose track:" + newline + ...
        " 1: Laguna Seca " + newline + ...
        " 2: Imola " + newline + ...
        " 3: YasNorth " + newline + ...
        " 0: No bridges");
    track = input("Choose track identifier: ");
end
switch track
    case 0
        BRIDGE_POSITION.X_MAP = [];
        BRIDGE_POSITION.Y_MAP = [];
    case 1
        BRIDGE_POSITION.X_MAP = [224.014, -66.365, 219.7, 219.4, 218.8, -65.6, -66.7, -61.9, 586.3, 591.4, 630.4, 451.6, 458.7, 190.6, 83.4, -63.4, -58.5, 86.2, 85.8, 183.5, 451.0, 452.9, 460.0, 623.7];
        BRIDGE_POSITION.Y_MAP = [-272.811, -348.844, -294.4, -287.3, -281.5, 1.7, 7.7, 4.8, -25.9, -26.3, -304.2, -650.1, -649.0, -534.1, -150.2, 14.8, 11.4, -151.5, -144.0, -532.2, -653.3, -656.4, -655.1, -303.9];
    case 2
        BRIDGE_POSITION.X_MAP = [249.0, -62.4, 741.1, 739.4, 258.6, 258.44, -59.9, -59.4, -59.4, -59.6, -8.4, -8.2, -8.0, -7.9, -498.1, -498.0, -497.9, -498.1, -700.2];
        BRIDGE_POSITION.Y_MAP = [150.8, -212.4, 217.0, 220.9, -210.0, -215.0, -218.3, 212.1, -211.8, -215.4, 199.7, 202.5, 205.4, 207.8, -647.5, -645.6, -642.4, -639.9, -459.9];      
    case 3
        BRIDGE_POSITION.X_MAP = [];
        BRIDGE_POSITION.Y_MAP = [];
    otherwise
        BRIDGE_POSITION.X_MAP = [];
        BRIDGE_POSITION.Y_MAP = [];
        warning('No bridge positions defined for this database.');
end
%% PROCESSING

if(ground_truth); gt_timestamp = (gt.timestamp - double(log.time_offset_nsec))*10^-9; end

% compute heading
if (~exist('rad1.idx','var'))
    for i = 1:rad1.max_det
        opp_pos = [rad1.x_map(:,i), rad1.y_map(:,i)];
        [opp_yaw, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
        rad1.yaw_map(:,i) = opp_yaw;
        rad1.idx(:,i) = opp_idx;
    end
end
if(compare)
    if (~exist('rad2.idx','var'))
        for i = 1:rad2.max_det
            opp_pos = [rad2.x_map(:,i), rad2.y_map(:,i)];
            [opp_yaw, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
            rad2.yaw_map(:,i) = opp_yaw;
            rad2.idx(:,i) = opp_idx;
        end
    end
end

% match timestamp
ego.timestamp = (log.estimation.stamp__tot)*10^9+double(log.time_offset_nsec);
rad1_stamp = rad1.sens_stamp*10^9 + double(log.time_offset_nsec);

[ego.clos_idxs, ego.matched_idxs] = ...
    find_closest_stamp(rad1_stamp, ego.timestamp);
ego.pos = nan(numel(rad1_stamp), 2);
ego.yaw = nan(numel(rad1_stamp), 1);
ego.v = nan(numel(rad1_stamp), 2);
ego.pos(ego.matched_idxs, 1) = log.estimation.x_cog(ego.clos_idxs);
ego.pos(ego.matched_idxs, 2) = log.estimation.y_cog(ego.clos_idxs);
ego.yaw(ego.matched_idxs) = log.estimation.heading(ego.clos_idxs);
ego.v(ego.matched_idxs, 1) = log.estimation.vx(ego.clos_idxs);
ego.v(ego.matched_idxs, 2) = log.estimation.vy(ego.clos_idxs);

if(compare)
    rad2_epoch = rad2.sens_stamp*10^9 + double(log2.time_offset_nsec);
    [ego2.clos_idxs, ego2.matched_idxs] = ...
        find_closest_stamp(rad2_epoch, ego.timestamp);
    if isempty(ego2.clos_idxs)
        error('No timestamp matched between ego and opp.');
    end
    ego2.pos = nan(numel(rad2_epoch), 2);
    ego2.yaw = nan(numel(rad2_epoch), 1);
    ego2.v = nan(numel(rad2_epoch), 2);
    ego2.pos(ego2.matched_idxs, 1) = log.estimation.x_cog(ego2.clos_idxs);
    ego2.pos(ego2.matched_idxs, 2) = log.estimation.y_cog(ego2.clos_idxs);
    ego2.yaw(ego2.matched_idxs) = log.estimation.heading(ego2.clos_idxs);
    ego2.v(ego2.matched_idxs, 1) = log.estimation.vx(ego2.clos_idxs);
    ego2.v(ego2.matched_idxs, 2) = log.estimation.vy(ego2.clos_idxs);
end

% compute opp speed
for i = 1:rad1.max_det
    opp_yaw = rad1.yaw_map(:,i);
    rad1.vx(:,i) = compensate_ego_speed(ego.pos, ego.v, ego.yaw, opp_yaw, ...
                        rad1.rho_dot(:,i), rad1.x_map(:,i), rad1.y_map(:,i));
end
rad1.range = sqrt(rad1.x_rel.^2 + rad1.y_rel.^2);

if(compare)
    for i = 1:max(rad2.max_det,1)
        opp_yaw = rad2.yaw_map(:,i);
        rad2.vx(:,i) = compensate_ego_speed(ego2.pos, ego2.v, ego2.yaw, opp_yaw, ...
                             rad2.rho_dot(:,i), rad2.x_map(:,i), rad2.y_map(:,i));
    end
    rad2_sens_stamp = rad2.sens_stamp + double(log2.time_offset_nsec)*10^-9 - double(log.time_offset_nsec)*10^-9; 
    rad2.range = sqrt(rad2.x_rel.^2 + rad2.y_rel.^2);
end

% detection number
rad1 = sum_radar_counts(rad1, 4);
if(compare); rad2 = sum_radar_counts(rad2, 4); end

% Ground-truth classification.
rad1.valid_detection = isfinite(rad1.sens_stamp) & ...
    isfinite(rad1.x_map) & isfinite(rad1.y_map);
rad1.correct = false(size(rad1.valid_detection));
rad1.false_positive = false(size(rad1.valid_detection));

if compare
    rad2.valid_detection = isfinite(rad2_sens_stamp) & ...
        isfinite(rad2.x_map) & isfinite(rad2.y_map);
    rad2.correct = false(size(rad2.valid_detection));
    rad2.false_positive = false(size(rad2.valid_detection));
end

if ground_truth
    [rad1.correct, rad1.false_positive, rad1.valid_detection] = ...
        classify_lidar_detections(rad1.sens_stamp, rad1.x_map, ...
        rad1.y_map, gt_timestamp, gt.x_map, gt.y_map, ...
        TEMPORAL_GATE, SPATIAL_GATE);

    if compare
        [rad2.correct, rad2.false_positive, rad2.valid_detection] = ...
            classify_lidar_detections(rad2_sens_stamp, rad2.x_map, ...
            rad2.y_map, gt_timestamp, gt.x_map, gt.y_map, ...
            TEMPORAL_GATE, SPATIAL_GATE);
    end
end

countData.count = sum(rad1.correct, 2);
countData = sum_radar_counts(countData, 4);
rad1.correct_count = countData.count;
countData.count = sum(rad1.false_positive, 2);
countData = sum_radar_counts(countData, 4);
rad1.false_positive_count = countData.count;
if compare
    countData.count = sum(rad2.correct, 2);
    countData = sum_radar_counts(countData, 4);
    rad2.correct_count = countData.count;
    countData.count = sum(rad2.false_positive, 2);
    countData = sum_radar_counts(countData, 4);
    rad2.false_positive_count = countData.count;
end

% liveliness
% liv_idx = find_radar_indices(log);
% liveliness_stamp = log.liveliness__state.bag_stamp;


% %% LATENCY FIGURE
% figure('name','Latency')
% tiledlayout(3,1,'Padding','compact');
% 
% axes(f) = nexttile; f=f+1;
% hold on;
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(1)), 'DisplayName','RF');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(3)), 'DisplayName','RR');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(5)), 'DisplayName','RL');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(7)), 'DisplayName','RB');
% grid on; title('radar points rate [s]'); legend; xlim(x_lim);
% 
% axes(f) = nexttile; f=f+1;
% hold on;
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(2)), 'DisplayName','RF');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(4)), 'DisplayName','RR');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(6)), 'DisplayName','RL');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(8)), 'DisplayName','RB');
% grid on; title('radar udp packages rate [s]'); legend; xlim(x_lim);
% 
% axes(f) = nexttile; f=f+1;
% hold on;
% plot(rad1.stamp,rad1.stamp-rad1.sens_stamp, 'Color', col.radar,'DisplayName',NAME_1);
% grid on; title('radar clustering latency [s]'); legend; xlim(x_lim);


%% STATE FIGURE COG
figure('name','Detections - CoG')
tiledlayout(2,1,'Padding','compact');

% pos x
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.x_rel, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.x_rel, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.x_rel, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('x rel [m]'); xlim(x_lim); ylim([-200 200]); legend show;

% pos y
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.y_rel, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.y_rel, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.y_rel, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('y rel [m]'); xlim(x_lim); ylim([-100 100]); legend show;

%% STATE FIGURE RANGE
figure('name','Detections - Range')
tiledlayout(1,1,'Padding','compact');

% rho 
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.range, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.range, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.rho, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('range [m]'); xlim(x_lim); ylim([0 200]); legend show;


%% STATE FIGURE MAP
figure('name','Detections - Map')
tiledlayout(2,1,'Padding','compact');

% pos X
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.x_map, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.x_map, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.x_map, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('x map [m]'); xlim(x_lim); legend show;

% pos y
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.y_map, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.y_map, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.y_map, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('y map [m]'); xlim(x_lim); legend show;

%% STATE FIGURE SPEED
figure('name','Detections - Speed')
tiledlayout(2,1,'Padding','compact');

% rho dot
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.rho_dot, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.rho_dot, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.rho_dot, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('rho dot [m/s]'); xlim(x_lim); legend show;

% vx
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plotClassifiedDetections(rad1.sens_stamp, rad1.vx, rad1.max_det, ...
    col.radar, NAME_1, sz, ground_truth, rad1.correct, ...
    rad1.false_positive);
if compare
    plotClassifiedDetections(rad2_sens_stamp, rad2.vx, rad2.max_det, ...
        col.radar2, NAME_2, sz, ground_truth, rad2.correct, ...
        rad2.false_positive);
end
if(ground_truth); plot(gt_timestamp, gt.speed, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('vx [m/s]'); xlim(x_lim); ylim([0 inf]); legend show;

%% DETECTION COUNT FIGURE

countFigure = figure('Name', 'Detections - Count');
tiledlayout(1, 1, 'Padding', 'compact');

axes(f) = nexttile([1, 1]); f = f + 1;
countAxes = axes(f - 1);
hold on;
stairs(rad1.sens_stamp, rad1.count, 'Color', col.radar, ...
    'DisplayName', NAME_1 + " - total");
if ground_truth
    stairs(rad1.sens_stamp, rad1.correct_count, '-o', ...
        'Color', col.radar, 'MarkerFaceColor', col.radar, ...
        'MarkerSize', sz, 'DisplayName', NAME_1 + " - TP");
    stairs(rad1.sens_stamp, rad1.false_positive_count, '--o', ...
        'Color', col.radar, 'MarkerFaceColor', 'none', ...
        'MarkerSize', sz, 'DisplayName', NAME_1 + " - FP");
end
if compare
    stairs(rad2_sens_stamp, rad2.count, 'Color', col.radar2, ...
        'DisplayName', NAME_2 + " - total");
    if ground_truth
        stairs(rad2_sens_stamp, rad2.correct_count, '-o', ...
            'Color', col.radar2, 'MarkerFaceColor', col.radar2, ...
            'MarkerSize', sz, 'DisplayName', NAME_2 + " - TP");
        stairs(rad2_sens_stamp, rad2.false_positive_count, '--o', ...
            'Color', col.radar2, 'MarkerFaceColor', 'none', ...
            'MarkerSize', sz, 'DisplayName', NAME_2 + " - FP");
    end
    comparisonStats.stamp = rad2_sens_stamp;
    comparisonStats.valid = rad2.valid_detection;
    comparisonStats.correct = rad2.correct;
    comparisonStats.false_positive = rad2.false_positive;
else
    comparisonStats.stamp = [];
    comparisonStats.valid = [];
    comparisonStats.correct = [];
    comparisonStats.false_positive = [];
end
primaryStats.stamp = rad1.sens_stamp;
primaryStats.valid = rad1.valid_detection;
primaryStats.correct = rad1.correct;
primaryStats.false_positive = rad1.false_positive;
grid on;
xlabel('timestamp [s]');
ylabel('detections [#]');
xlim(x_lim);
legend show;
title('Detection count over time');
if(ground_truth)
    countStatisticsBox = annotation(countFigure, 'textbox', ...
        STATISTICS_BOX_POSITION, 'Tag', 'DetectionCountStatistics', ...
        'FitBoxToText', 'off', 'BackgroundColor', 'white', ...
        'EdgeColor', 'black', 'Margin', 4, ...
        'VerticalAlignment', 'top', 'Interpreter', 'none');
updateDetectionTotal(countAxes, countStatisticsBox, primaryStats, NAME_1, ...
    ground_truth, compare, comparisonStats, NAME_2);
countListener = addlistener(countAxes, 'XLim', 'PostSet', ...
    @(~, ~) updateDetectionTotal(countAxes, countStatisticsBox, ...
    primaryStats, NAME_1, ground_truth, compare, comparisonStats, NAME_2));
setappdata(countFigure, 'DetectionCountXLimListener', countListener);
end

linkaxes(axes, 'x');


%% MAP GUI
fig = figure('Name','MAP');

c = uicontrol('Style','pushbutton');
c.String = {'Refresh'};
c.Callback = @refreshTimeButtonPushed;

function refreshTimeButtonPushed(src,event)
    axes = evalin('base', 'axes');
    traj_db = evalin('base', 'trajDatabase');
    ground_truth = evalin('base', 'ground_truth');
    compare = evalin('base', 'compare');
    col = evalin('base', 'col');
    rad1 = evalin('base', 'rad1');
    bridge_position = evalin('base', 'BRIDGE_POSITION');
    primary_name = evalin('base', 'NAME_1');
    comparison_name = evalin('base', 'NAME_2');
    statistics_box_position = evalin('base', 'STATISTICS_BOX_POSITION');
    if(ground_truth); gt=evalin('base','gt'); end
    if(ground_truth); gt_timestamp = evalin('base','gt_timestamp'); end
    if(compare); rad2 = evalin('base', 'rad2'); end
    if(compare); rad2_sens_stamp = evalin('base', 'rad2_sens_stamp'); end
    
    t_lim=xlim(axes(1));
    rad1_time_idx = rad1.sens_stamp > t_lim(1) & ...
        rad1.sens_stamp < t_lim(2);
    if(compare)
        rad2_time_idx = rad2_sens_stamp > t_lim(1) & ...
            rad2_sens_stamp < t_lim(2);
    end
    if(ground_truth)
        gt_idx = gt_timestamp > t_lim(1) & gt_timestamp < t_lim(2);
    end

    figure_handle = ancestor(src, 'figure');
    map_axes = subplot(1, 1, 1, 'Parent', figure_handle);
    cla(map_axes, 'reset');
    title(map_axes, 'map');
    hold(map_axes, 'on');
    grid(map_axes, 'on');
    xlabel(map_axes, 'x[m]');
    ylabel(map_axes, 'y[m]');
    axis(map_axes, 'equal');

    % plot track lines
    id_left = length(traj_db) - 2;
    id_right = length(traj_db) - 1;
    plot(traj_db(id_left).x, traj_db(id_left).y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
    plot(traj_db(id_right).x, traj_db(id_right).y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');

    if ground_truth
        rad1_tp_idx = rad1_time_idx & rad1.correct;
        rad1_fp_idx = rad1_time_idx & rad1.false_positive;
        scatter(map_axes, rad1.x_map(rad1_tp_idx), rad1.y_map(rad1_tp_idx), ...
            20, 'filled', 'MarkerEdgeColor', col.radar, ...
            'MarkerFaceColor', col.radar, ...
            'DisplayName', primary_name + " - TP");
        scatter(map_axes, rad1.x_map(rad1_fp_idx), rad1.y_map(rad1_fp_idx), ...
            20, 'o', 'MarkerEdgeColor', col.radar, ...
            'MarkerFaceColor', 'none', ...
            'DisplayName', primary_name + " - FP");
    else
        rad1_x_map = rad1.x_map(rad1_time_idx, :);
        rad1_y_map = rad1.y_map(rad1_time_idx, :);
        scatter(map_axes, rad1_x_map(:), rad1_y_map(:), 20, 'filled', ...
            'MarkerEdgeColor', col.radar, 'MarkerFaceColor', col.radar, ...
            'DisplayName', primary_name);
    end
    if compare
        if ground_truth
            rad2_tp_idx = rad2_time_idx & rad2.correct;
            rad2_fp_idx = rad2_time_idx & rad2.false_positive;
            scatter(map_axes, rad2.x_map(rad2_tp_idx), ...
                rad2.y_map(rad2_tp_idx), 20, 'filled', ...
                'MarkerEdgeColor', col.radar2, ...
                'MarkerFaceColor', col.radar2, ...
                'DisplayName', comparison_name + " - TP");
            scatter(map_axes, rad2.x_map(rad2_fp_idx), ...
                rad2.y_map(rad2_fp_idx), 20, 'o', ...
                'MarkerEdgeColor', col.radar2, ...
                'MarkerFaceColor', 'none', ...
                'DisplayName', comparison_name + " - FP");
        else
            rad2_x_map = rad2.x_map(rad2_time_idx, :);
            rad2_y_map = rad2.y_map(rad2_time_idx, :);
            scatter(map_axes, rad2_x_map(:), rad2_y_map(:), 20, 'filled', ...
                'MarkerEdgeColor', col.radar2, ...
                'MarkerFaceColor', col.radar2, ...
                'DisplayName', comparison_name);
        end
    end
    if ground_truth
        plot(map_axes, gt.x_map(gt_idx), gt.y_map(gt_idx), ...
            'Color', col.ref, 'DisplayName', 'Ground truth');
    end

    % Draw draggable bridge regions using THRESHOLD as the radius in meters.
    circle_angle = linspace(0, 2*pi, 64);
    for bridge_idx = 1:numel(bridge_position.X_MAP)
        if bridge_idx == 1
            handle_visibility = 'on';
        else
            handle_visibility = 'off';
        end

        patch( ...
            bridge_position.X_MAP(bridge_idx) + bridge_position.THRESHOLD*cos(circle_angle), ...
            bridge_position.Y_MAP(bridge_idx) + bridge_position.THRESHOLD*sin(circle_angle), ...
            'red', ...
            'FaceAlpha', 0.3, ...
            'EdgeColor', 'black', ...
            'HandleVisibility', handle_visibility, ...
            'DisplayName', 'Bridge positions', ...
            'PickableParts', 'all', ...
            'HitTest', 'on', ...
            'ButtonDownFcn', @(circle, event) startBridgeDrag(circle, event, bridge_idx));
    end

    primary_stats.stamp = rad1.sens_stamp;
    primary_stats.valid = rad1.valid_detection;
    primary_stats.correct = rad1.correct;
    primary_stats.false_positive = rad1.false_positive;
    if compare
        comparison_stats.stamp = rad2_sens_stamp;
        comparison_stats.valid = rad2.valid_detection;
        comparison_stats.correct = rad2.correct;
        comparison_stats.false_positive = rad2.false_positive;
    else
        comparison_stats = struct();
    end

    map_statistics_box = findall(figure_handle, ...
        'Tag', 'MapDetectionStatistics');
    if isempty(map_statistics_box)
        map_statistics_box = annotation(figure_handle, 'textbox', ...
            statistics_box_position, 'Tag', 'MapDetectionStatistics', ...
            'FitBoxToText', 'off', 'BackgroundColor', 'white', ...
            'EdgeColor', 'black', 'Margin', 4, ...
            'VerticalAlignment', 'top', 'Interpreter', 'none');
    end
    map_statistics_box.String = formatCombinedDetectionStatistics( ...
        primary_stats, primary_name, ground_truth, compare, ...
        comparison_stats, comparison_name, t_lim);

    legend(map_axes, 'show');
end

function startBridgeDrag(circle, ~, bridge_idx)
    fig = ancestor(circle, 'figure');
    if ~strcmp(fig.SelectionType, 'normal')
        return;
    end

    ax = ancestor(circle, 'axes');
    bridge_position = evalin('base', 'BRIDGE_POSITION');
    center = [ ...
        bridge_position.X_MAP(bridge_idx), ...
        bridge_position.Y_MAP(bridge_idx)];
    cursor_position = ax.CurrentPoint(1, 1:2);

    drag_state.circle = circle;
    drag_state.axes = ax;
    drag_state.bridge_idx = bridge_idx;
    drag_state.cursor_offset = center - cursor_position;
    drag_state.circle_x_offset = circle.XData - center(1);
    drag_state.circle_y_offset = circle.YData - center(2);
    drag_state.center = center;

    setappdata(fig, 'BridgeDragState', drag_state);
    fig.Pointer = 'fleur';
    fig.WindowButtonMotionFcn = @moveBridgeCircle;
    fig.WindowButtonUpFcn = @finishBridgeDrag;
end

function moveBridgeCircle(fig, ~)
    if ~isappdata(fig, 'BridgeDragState')
        return;
    end

    drag_state = getappdata(fig, 'BridgeDragState');
    cursor_position = drag_state.axes.CurrentPoint(1, 1:2);
    drag_state.center = cursor_position + drag_state.cursor_offset;

    drag_state.circle.XData = drag_state.center(1) + drag_state.circle_x_offset;
    drag_state.circle.YData = drag_state.center(2) + drag_state.circle_y_offset;
    setappdata(fig, 'BridgeDragState', drag_state);
    drawnow limitrate nocallbacks;
end

function finishBridgeDrag(fig, ~)
    if ~isappdata(fig, 'BridgeDragState')
        return;
    end

    moveBridgeCircle(fig, []);
    drag_state = getappdata(fig, 'BridgeDragState');

    bridge_position = evalin('base', 'BRIDGE_POSITION');
    bridge_position.X_MAP(drag_state.bridge_idx) = drag_state.center(1);
    bridge_position.Y_MAP(drag_state.bridge_idx) = drag_state.center(2);
    assignin('base', 'BRIDGE_POSITION', bridge_position);

    fprintf('\nBridge circle %d moved:\n', drag_state.bridge_idx);
    fprintf('BRIDGE_POSITION.X_MAP(%d) = %.3f;\n', ...
        drag_state.bridge_idx, drag_state.center(1));
    fprintf('BRIDGE_POSITION.Y_MAP(%d) = %.3f;\n', ...
        drag_state.bridge_idx, drag_state.center(2));

    rmappdata(fig, 'BridgeDragState');
    fig.Pointer = 'arrow';
    fig.WindowButtonMotionFcn = '';
    fig.WindowButtonUpFcn = '';
end

function plotClassifiedDetections(stamp, values, maxDet, color, name, ...
        markerSize, groundTruthEnabled, correctMask, falsePositiveMask)
    values = safe_cols(values, maxDet);
    if isvector(stamp) && numel(stamp) == size(values, 1)
        stamp = repmat(stamp(:), 1, size(values, 2));
    else
        stamp = safe_cols(stamp, maxDet);
    end

    if groundTruthEnabled
        correctMask = safe_cols(correctMask, maxDet);
        falsePositiveMask = safe_cols(falsePositiveMask, maxDet);

        correctValues = values;
        correctValues(~correctMask) = NaN;
        falsePositiveValues = values;
        falsePositiveValues(~falsePositiveMask) = NaN;

        plot(stamp, correctValues, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'HandleVisibility', 'off');
        plot(stamp, falsePositiveValues, 'o', ...
            'MarkerFaceColor', 'none', 'MarkerEdgeColor', color, ...
            'MarkerSize', markerSize, 'HandleVisibility', 'off');
        plot(NaN, NaN, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'DisplayName', name + " - TP");
        plot(NaN, NaN, 'o', 'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'DisplayName', name + " - FP");
    else
        plot(stamp, values, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'HandleVisibility', 'off');
        plot(NaN, NaN, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'DisplayName', name);
    end
end

function updateDetectionTotal(countAxes, statisticsBox, primaryStats, ...
        primaryName, groundTruthEnabled, compareLogs, comparisonStats, ...
        comparisonName)
    timeLimits = xlim(countAxes);
    statisticsBox.String = formatCombinedDetectionStatistics( ...
        primaryStats, primaryName, groundTruthEnabled, compareLogs, ...
        comparisonStats, comparisonName, timeLimits);
end

function combinedText = formatCombinedDetectionStatistics(primaryStats, ...
        primaryName, groundTruthEnabled, compareLogs, comparisonStats, ...
        comparisonName, timeLimits)
    primaryText = formatDetectionStatistics(primaryStats, primaryName, ...
        timeLimits, groundTruthEnabled);
    if compareLogs
        comparisonText = formatDetectionStatistics(comparisonStats, ...
            comparisonName, timeLimits, groundTruthEnabled);
        combinedText = sprintf('%s\n%s', primaryText, comparisonText);
    else
        combinedText = primaryText;
    end
end

function statisticsText = formatDetectionStatistics(statistics, name, ...
        timeLimits, groundTruthEnabled)
    selectedWindow = statistics.stamp >= timeLimits(1) & ...
        statistics.stamp <= timeLimits(2);
    total = nnz(statistics.valid & selectedWindow);

    if groundTruthEnabled
        truePositive = nnz(statistics.correct & selectedWindow);
        falsePositive = nnz(statistics.false_positive & selectedWindow);
        truePositiveRatio = 100 * truePositive / max(total, 1);
        falsePositiveRatio = 100 * falsePositive / max(total, 1);
        statisticsText = sprintf( ...
            ['%s - Total: %d\n' ...
            'TP: %d (%.1f%%) | FP: %d (%.1f%%)'], ...
            name, total, truePositive, truePositiveRatio, falsePositive, ...
            falsePositiveRatio);
    else
        statisticsText = sprintf('%s - Total: %d', name, total);
    end
end

close all
clearvars -except log log2 trajDatabase gt ego ego2 rad1 rad2 track

ground_truth = true;
compare      = false;

BRIDGE_POSITION.THRESHOLD = 3;
TEMPORAL_GATE = 0.1; % [s]
SPATIAL_GATE = 2.0;  % [m]
STATISTICS_BOX_POSITION = [0.14 0.78 0.32 0.16];

NAME_1 = "Radar";
NAME_2 = "Radar - 2";

%#ok<*UNRCH>
%#ok<*INUSD>

%% PATHS

proj = currentProject();
addpath(fullfile(proj.RootFolder,'src','perception','utils'));

%% LOAD FILES

% Load database.
if ~exist('trajDatabase','var')
    trajDatabase = choose_database();
    if isempty(trajDatabase)
        error('No database selected');
    else
        load(trajDatabase, 'trajDatabase');
    end
end

% Load primary log.
if ~exist('log','var')
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    if isequal(file,0)
        error('No log selected');
    end
    load(fullfile(path,file), 'log');
end

% Load comparison log.
if compare && ~exist('log2','var')
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log 2');
    if isequal(file,0)
        error('No comparison log selected');
    end
    tmp = load(fullfile(path,file), 'log');
    log2 = tmp.log;
end

% Load ground truth.
if ground_truth && ~exist('gt','var')
    [file,path] = uigetfile(fullfile(get_gt_dir(),'*.mat'), ...
        'Load ground truth log');
    if isequal(file,0)
        error('No ground truth log selected');
    end
    tmp = load(fullfile(path,file), 'out');
    gt = tmp.out;
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime', ...
    'TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% NAMING

set(0,'DefaultFigureWindowStyle','docked');
set(0,'DefaultFigureVisible','on');
set(0,'DefaultTextInterpreter','none');
set(0,'DefaultLegendInterpreter','none');
set(0,'DefaultLineLineWidth',2);

col.radar = '#0072BD';
col.radar2 = '#FF0000';
col.ref = '#000000';
sz = 3; % Marker size
f = 1;
ax = gobjects(0);
x_lim = [0 inf];

%% LOAD DATA

rad1 = get_radar_fields(log);
if compare
    rad2 = get_radar_fields(log2);
end

if ~exist('track','var')
    track = -1;
end
tracks = [0,1,2,3];
while ~ismember(track,tracks)
    disp("Choose track:" + newline + ...
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
        BRIDGE_POSITION.X_MAP = [ ...
            224.014, -66.365, 219.7, 219.4, 218.8, -65.6, -66.7, ...
            -61.9, 586.3, 591.4, 630.4, 451.6, 458.7, 190.6, 83.4, ...
            -63.4, -58.5, 86.2, 85.8, 183.5, 451.0, 452.9, 460.0, 623.7];
        BRIDGE_POSITION.Y_MAP = [ ...
            -272.811, -348.844, -294.4, -287.3, -281.5, 1.7, 7.7, ...
            4.8, -25.9, -26.3, -304.2, -650.1, -649.0, -534.1, ...
            -150.2, 14.8, 11.4, -151.5, -144.0, -532.2, -653.3, ...
            -656.4, -655.1, -303.9];
    case 2
        BRIDGE_POSITION.X_MAP = [ ...
            249.0, -62.4, 741.1, 739.4, 258.6, 258.44, -59.9, -59.4, ...
            -59.4, -59.6, -8.4, -8.2, -8.0, -7.9, -498.1, -498.0, ...
            -497.9, -498.1, -700.2];
        BRIDGE_POSITION.Y_MAP = [ ...
            150.8, -212.4, 217.0, 220.9, -210.0, -215.0, -218.3, ...
            212.1, -211.8, -215.4, 199.7, 202.5, 205.4, 207.8, ...
            -647.5, -645.6, -642.4, -639.9, -459.9];
    case 3
        BRIDGE_POSITION.X_MAP = [];
        BRIDGE_POSITION.Y_MAP = [];
    otherwise
        BRIDGE_POSITION.X_MAP = [];
        BRIDGE_POSITION.Y_MAP = [];
        warning('No bridge positions defined for this database.');
end

%% PROCESSING

if ground_truth
    gt_timestamp = (gt.timestamp - double(log.time_offset_nsec))*1e-9;
end

% Compute opponent heading.
if ~isfield(rad1,'idx')
    for i = 1:rad1.max_det
        opp_pos = [rad1.x_map(:,i),rad1.y_map(:,i)];
        [opp_yaw,opp_idx] = get_heading(opp_pos,trajDatabase,-1);
        rad1.yaw_map(:,i) = opp_yaw;
        rad1.idx(:,i) = opp_idx;
    end
end
if compare && ~isfield(rad2,'idx')
    for i = 1:rad2.max_det
        opp_pos = [rad2.x_map(:,i),rad2.y_map(:,i)];
        [opp_yaw,opp_idx] = get_heading(opp_pos,trajDatabase,-1);
        rad2.yaw_map(:,i) = opp_yaw;
        rad2.idx(:,i) = opp_idx;
    end
end

% Match radar and ego timestamps.
ego.timestamp = log.estimation.stamp__tot*1e9 + ...
    double(log.time_offset_nsec);
rad1_stamp = rad1.sens_stamp*1e9 + double(log.time_offset_nsec);
[ego.clos_idxs,ego.matched_idxs] = ...
    find_closest_stamp(rad1_stamp,ego.timestamp);
ego.pos = nan(numel(rad1_stamp),2);
ego.yaw = nan(numel(rad1_stamp),1);
ego.v = nan(numel(rad1_stamp),2);
ego.pos(ego.matched_idxs,1) = log.estimation.x_cog(ego.clos_idxs);
ego.pos(ego.matched_idxs,2) = log.estimation.y_cog(ego.clos_idxs);
ego.yaw(ego.matched_idxs) = log.estimation.heading(ego.clos_idxs);
ego.v(ego.matched_idxs,1) = log.estimation.vx(ego.clos_idxs);
ego.v(ego.matched_idxs,2) = log.estimation.vy(ego.clos_idxs);

if compare
    rad2_epoch = rad2.sens_stamp*1e9 + double(log2.time_offset_nsec);
    [ego2.clos_idxs,ego2.matched_idxs] = ...
        find_closest_stamp(rad2_epoch,ego.timestamp);
    if isempty(ego2.clos_idxs)
        error('No timestamp matched between ego and opponent.');
    end
    ego2.pos = nan(numel(rad2_epoch),2);
    ego2.yaw = nan(numel(rad2_epoch),1);
    ego2.v = nan(numel(rad2_epoch),2);
    ego2.pos(ego2.matched_idxs,1) = log.estimation.x_cog(ego2.clos_idxs);
    ego2.pos(ego2.matched_idxs,2) = log.estimation.y_cog(ego2.clos_idxs);
    ego2.yaw(ego2.matched_idxs) = log.estimation.heading(ego2.clos_idxs);
    ego2.v(ego2.matched_idxs,1) = log.estimation.vx(ego2.clos_idxs);
    ego2.v(ego2.matched_idxs,2) = log.estimation.vy(ego2.clos_idxs);
end

% Compute opponent speed.
for i = 1:rad1.max_det
    opp_yaw = rad1.yaw_map(:,i);
    rad1.vx(:,i) = compensate_ego_speed(ego.pos,ego.v,ego.yaw, ...
        opp_yaw,rad1.rho_dot(:,i),rad1.x_map(:,i),rad1.y_map(:,i));
end
rad1.range = hypot(rad1.x_rel,rad1.y_rel);

if compare
    for i = 1:max(rad2.max_det,1)
        opp_yaw = rad2.yaw_map(:,i);
        rad2.vx(:,i) = compensate_ego_speed(ego2.pos,ego2.v,ego2.yaw, ...
            opp_yaw,rad2.rho_dot(:,i),rad2.x_map(:,i),rad2.y_map(:,i));
    end
    rad2_sens_stamp = rad2.sens_stamp + ...
        double(log2.time_offset_nsec)*1e-9 - ...
        double(log.time_offset_nsec)*1e-9;
    rad2.range = hypot(rad2.x_rel,rad2.y_rel);
end

% Detection count.
rad1 = sum_radar_counts(rad1,4);
if compare
    rad2 = sum_radar_counts(rad2,4);
end

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
    [rad1.correct,rad1.false_positive,rad1.valid_detection] = ...
        classify_lidar_detections(rad1.sens_stamp,rad1.x_map, ...
        rad1.y_map,gt_timestamp,gt.x_map,gt.y_map, ...
        TEMPORAL_GATE,SPATIAL_GATE);

    if compare
        [rad2.correct,rad2.false_positive,rad2.valid_detection] = ...
            classify_lidar_detections(rad2_sens_stamp,rad2.x_map, ...
            rad2.y_map,gt_timestamp,gt.x_map,gt.y_map, ...
            TEMPORAL_GATE,SPATIAL_GATE);
    end
end

count_data.count = sum(rad1.correct,2);
count_data = sum_radar_counts(count_data,4);
rad1.correct_count = count_data.count;
count_data.count = sum(rad1.false_positive,2);
count_data = sum_radar_counts(count_data,4);
rad1.false_positive_count = count_data.count;

if compare
    count_data.count = sum(rad2.correct,2);
    count_data = sum_radar_counts(count_data,4);
    rad2.correct_count = count_data.count;
    count_data.count = sum(rad2.false_positive,2);
    count_data = sum_radar_counts(count_data,4);
    rad2.false_positive_count = count_data.count;
end

%% PLOTTING

radar_timeseries;
detection_count;
radar_stiffness;
radar_map;

linkaxes(ax,'x');

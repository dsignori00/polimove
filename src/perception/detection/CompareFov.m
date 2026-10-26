clc
close all
clearvars -except log log2 log_ref trajDatabase

compare      = false;
ground_truth = false;

NAME_1 = "";
NAME_2 = "old - ";

%#ok<*UNRCH>
%#ok<*INUSD>

%% Paths
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir,'shared','figs'));
addpath(fullfile(get_tracking_dir(),'TargetTrackingAnalysis','figs'));

%% LOAD FILES

%load database
if(~exist('trajDatabase','var'))
    trajDatabase = choose_database();
    if(isempty(trajDatabase))
        error('No database selected');
    else
        load(trajDatabase, 'trajDatabase');
    end
end

% load log
if (~exist('log','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    load(fullfile(path,file), 'log');
end

% load log2
if (compare && ~exist('log2','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log 2');
    tmp = load(fullfile(path,file), 'log');
    log2 = tmp.log;
end

if(ground_truth)
    if (~exist('log_ref','var'))
        [file,path] = uigetfile(fullfile(get_gt_dir(),'*.mat'),'Load ground truth log');
        tmp = load(fullfile(path,file), 'out');
        log_ref = tmp.out;
    end
else
    log_ref = [];
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% OPTIONS
graphics_options;
col.lidar        = colors.green{2};
col.lidar2       = colors.green{1};
col.radar        = colors.blue{2};
col.radar2       = colors.blue{1};
col.camera       = colors.yellow{2};
col.camera2      = colors.yellow{1};
col.pp = colors.orange{2};
col.pointpillars2= colors.orange{1};
col.ref          = colors.black;

sz=3; % Marker size
f=1;
x_lim = [0 inf];

%% LOAD DATA

[lid_clust, rad_clust, cam_yolo, lid_pp, bevfus] = load_perception(log);
gt = load_ref(log, false, ground_truth, log_ref);
cam_yolo.sens_stamp(cam_yolo.sens_stamp < 0) = NaN;

if(compare)
    [lid_clust2, rad_clust2, cam_yolo2, lid_pp2, bevfus2] = load_perception(log2);
    cam_yolo2.sens_stamp(cam_yolo2.sens_stamp < 0) = NaN;
    lid_clust2.sens_stamp = lid_clust2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    rad_clust2.sens_stamp = rad_clust2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    cam_yolo2.sens_stamp = cam_yolo2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    lid_pp2.sens_stamp = lid_pp2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
end

%% SENSORS

sensors = { ...
    struct('s', lid_clust, 'col', col.lidar, 'name', NAME_1+"lidar", 'fov', [10 179]), ...
    struct('s', rad_clust, 'col', col.radar, 'name', NAME_1+"radar", 'fov', [90 0; 90 180]), ...
    struct('s', lid_pp, 'col', col.pp, 'name', NAME_1+"pointpillars"), ...
    struct('s', cam_yolo, 'col', col.camera, 'name', NAME_1+"yolo", 'fov', [85 0; 85 180]), ...
};

if(compare)
    sensors = [sensors, { ...
        struct('s', lid_clust2, 'col', col.lidar2, 'name', NAME_2+"lidar", 'fov', [10 179]), ...
        struct('s', rad_clust2, 'col', col.radar2, 'name', NAME_2+"radar", 'fov', [90 0; 90 180]), ...
        struct('s', lid_pp2, 'col', col.pointpillars2, 'name', NAME_2+"pointpillars"), ...
        struct('s', cam_yolo2, 'col', col.camera2, 'name', NAME_2+"yolo", 'fov', [85 0; 85 180]), ...
    }];
end

%% PLOTTING

% Settings required by the shared range script (no target tracking).
tt_exists = false;
tt2_exists = false;
compare2 = false;
use_ref = ground_truth;
use_sim_ref = false;

range;
xlim(ax(1),x_lim);
xlabel(ax(end),'time [s]');
linkaxes(ax,'x');
fov;

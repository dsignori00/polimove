close all; clc;
clearvars -except colors log log_2 log_3 log_ref trajDatabase

link_axes_mode = 'all'; % 'none', 'figure', or 'all'

use_ref     = true;
use_sim_ref = false;
compare     = false;   % LOAD NOT IMPLEMENTED
compare2    = false;   % LOAD NOT IMPLEMENTED

opp_idx     = 1;
err_thr     = 10;
err_stats = {'yaw_map','vx','ax'};

%#ok<*UNRCH>
%#ok<*INUSD>

%% Paths
normal_path = get_bags_dir();
opp_dir = get_gt_dir();
tracking_dir = get_tracking_dir();
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir,'shared','figs'));
addpath(fullfile(tracking_dir,'TargetTrackingAnalysis','figs'));

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
    [file,path] = uigetfile(fullfile(normal_path,'*.mat'),'Load log');
    load(fullfile(path,file));
end
name1 = 'new';

% load log ref
if(use_ref)
    if  (~exist('log_ref','var') || isempty(log_ref))
        [file,path] = uigetfile(fullfile(opp_dir,'*.mat'),'Load ground truth');
        tmp = load(fullfile(path,file));
        log_ref = tmp.out;
        clearvars tmp;
    end
end
if(~exist('log_ref','var'))
    log_ref = [];
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% NAMING
graphics_options;
col.lidar   = colors.green{2};
col.pp      = colors.green{3};
col.radar   = [77 190 238] / 255;
col.conly  = colors.yellow{2};
col.cenh  = colors.orange{2};
col.bevfus  = colors.purple{2};
col.ref     = colors.black;
sz=3; % Marker size
f=1;
c = 0;
x_lim = [0 inf];

%% PARSING

[lid_clust, rad_clust, cam_yolo, lid_pp, bevfus, v2v] = load_perception(log);

[cam_only, cam_enh] = splitCamYolo(cam_yolo);

if(use_ref || use_sim_ref); gt = load_ref(log, use_sim_ref, use_ref, log_ref); end

% Disable TT plots from figures
tt_exists = exist('tt', 'var');

sensors = { ...
    struct('s', lid_clust, 'col', col.lidar,  'name', 'LD'), ...
    struct('s', rad_clust, 'col', col.radar,  'name', 'RD'), ...
    struct('s', lid_pp, 'col', col.pp, 'name', 'PP'), ...
    struct('s', cam_enh, 'col', col.cenh,  'name', 'Cenh'), ...
    struct('s', bevfus, 'col', col.bevfus,  'name', 'BevFus'), ...
};
%% PLOTTING

% info;
detections_counts;
latency;
state_map;
state_cog;
range;
map;
fov;

link_axes();

clc; close all; clearvars -except log log_ref trajDatabase

use_tt              = false;
use_sim_ref         = false;
show_error_series   = true;
search_correlations = false;
link_axes_mode      = 'all'; % 'none', 'figure', 'active', or 'all'

%#ok<*UNRCH>
%#ok<*INUSD>

%% LOAD FILES

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
    if isequal(file, 0)
        error('No log selected');
    end
    load(fullfile(path,file));
end

% load ref
if(~use_sim_ref)
    if (~exist('log_ref','var'))
        [file,path_dir] = uigetfile(fullfile(get_gt_dir(),'*.mat'),'Load ground truth mat');
        if isequal(file, 0)
            error('No ground truth selected');
        end
        tmp = load(fullfile(path_dir,file));
        fields = fieldnames(tmp);
        log_ref = tmp.(fields{1});
        clearvars tmp;
    end
else
    log_ref = [];
end


DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% NAMING
graphics_options;
col.lidar        = colors.green{2};
col.radar        = [77 190 238] / 255;
col.camera       = colors.yellow{2};
col.pp           = colors.orange{2};
col.tt           = colors.blue{2};
col.tt2          = colors.blue{1};
col.ref          = colors.black;
sz=3; % Marker size
f=1;
b=0;

err_thr = 10;
x_lim = [0 inf];
y_err_lim = [-err_thr err_thr];

%% LOAD DATA

[lid_clust, rad_clust, cam_yolo, lid_pp] = load_perception(log);
gt = load_ref(log, use_sim_ref, ~use_sim_ref, log_ref);
cam_yolo.sens_stamp(cam_yolo.sens_stamp < 0) = NaN;
cam_yolo.sens_stamp(cam_yolo.x_rel < 0) = NaN;

% ego
ego.speed_stamp = log.estimation.stamp__tot;
ego.speed = log.estimation.vx;

% target tracking
if use_tt
    tt = load_tt(log);
end


%% PROCESSING 

sensors = { ...
    struct('id', 'lid_clust', 's', lid_clust, 'col', col.lidar,   'name', 'lidar',  'has_rho_dot', false), ...
    struct('id', 'rad_clust', 's', rad_clust, 'col', col.radar,   'name', 'radar',  'has_rho_dot', true), ...
    struct('id', 'cam_yolo',  's', cam_yolo,  'col', col.camera,  'name', 'camera', 'has_rho_dot', false), ...
    struct('id', 'lid_pp',    's', lid_pp,    'col', col.pp,      'name', 'pp',     'has_rho_dot', false) ...
};

fields = {'stamp','sens_stamp','x_map','y_map','z_map','yaw_map','x_rel','y_rel','z_rel','yaw_rel'};
interpFields = {'x_map','y_map','x_rel','y_rel','yaw_map','yaw_rel'};

process_measures;

%% PLOTTING

latency;
time_series_map;
time_series_cog;
time_series_range;
time_series_errors;
% correlations;
fit_gaussian;
error_summary;
sensors_fov;
map;

link_axes();

clc; close all; clearvars -except log log_ref trajDatabase

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
%graphics_options;
%% graphics_options.m - definizione palette colori
colors.green  = {[0.40 0.75 0.40], [0.00 0.55 0.00]};
colors.yellow = {[0.95 0.85 0.20], [0.85 0.65 0.00]};
colors.orange = {[0.95 0.65 0.20], [0.90 0.45 0.00]};
colors.red    = {[0.90 0.40 0.40], [0.80 0.00 0.00]};
colors.matlab = {[0 0.4470 0.7410], [0.8500 0.3250 0.0980], [0.9290 0.6940 0.1250]};
colors.black  = [0 0 0];
col.lidar   = colors.green{2};
col.radar   = [77 190 238] / 255;
col.camera  = colors.yellow{2};
col.pp      = colors.orange{2};
col.v2v     = colors.red{2};
col.tt      = colors.matlab{1};
col.tt2     = colors.matlab{2};
col.tt3     = colors.matlab{3};
col.ref     = colors.black;
sz=3; % Marker size
f=1;
c = 0;
x_lim = [0 inf];



%% LOAD DATA

[lid_clust, rad_clust, cam_yolo, lid_pp] = load_perception(log);
gt = load_ref(log, use_sim_ref, ~use_sim_ref, log_ref);
cam_yolo.sens_stamp(cam_yolo.sens_stamp < 0) = NaN;
cam_yolo.sens_stamp(cam_yolo.x_rel < 0) = NaN;

% ego
ego.speed_stamp = log.estimation.stamp__tot;
ego.speed = log.estimation.vx;
% ego.rpm_stamp = log.vehicle_fbk.stamp__tot;
% ego.rpm = log.vehicle_fbk.engine_rpm;

% target tracking
tt.stamp = log.perception__opponents.stamp__tot;
%tt.count = log.perception__opponents.count;
tt.max_opp = max(tt.count);


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

%latency;
%time_series_map;
%time_series_cog;
%time_series_range;
%time_series_errors;
% correlations;
%fit_gaussian;
%error_summary;
%sensors_fov;
calibration_errors;
%map;




link_axes();

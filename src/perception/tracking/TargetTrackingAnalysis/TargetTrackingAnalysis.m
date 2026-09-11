close all; clc;
clearvars -except colors log log_2 log_3 log_ref trajDatabase

link_axes_mode = 'all'; % 'none', 'figure', or 'all'

use_ref     = true;
use_sim_ref = false;
compare     = true;
compare2    = false;

opp_idx     = 1;
err_thr     = 10;
err_stats = {'yaw_map','vx','ax'};

%#ok<*UNRCH>
%#ok<*INUSD>

%% Paths
normal_path = get_bags_dir();
opp_dir = get_gt_dir();
scriptDir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptDir,'func'));
addpath(fullfile(scriptDir,'figs'));

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
name1 = 'no radar';

% load log 2
if(compare)
    if (~exist('log_2','var'))
        [file,path] = uigetfile(fullfile(normal_path,'*.mat'),'Load log_2');
        if isequal(file, 0)  
        disp('User canceled file selection.');
        else
        tmp = load(fullfile(path,file));
        log_2 = tmp.log;
        clearvars tmp;
        end
    end
    name2 = 'all sensors';
end

% load log 3
if(compare2)
    if (~exist('log_3','var'))
        [file,path] = uigetfile(fullfile(normal_path,'*.mat'),'Load log_3');
        if isequal(file, 0)  
        disp('User canceled file selection.');
        else
        tmp = load(fullfile(path,file));
        log_3 = tmp.log;
        clearvars tmp;
        end
    end
    name3 = 'Q500';
end

% load log ref
if(use_ref)
    if  (~exist('log_ref','var'))
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

% %% NAMING
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

%% PARSING

[lid_clust, rad_clust, cam_yolo, lid_pp, v2v] = load_perception(log);
if(use_ref || use_sim_ref); gt = load_ref(log, use_sim_ref, use_ref, log_ref); end
tt = load_tt(log);
tt.col = col.tt;
tt.name = name1;
if(compare) 
    tt2 = load_tt(log_2); 
    tt2.stamp = tt2.stamp + double(log_2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    tt2.col = col.tt2;
    tt2.name = name2;
end
if(compare2)
    tt3 = load_tt(log_3); 
    tt3.stamp = tt3.stamp + double(log_3.time_offset_nsec-log.time_offset_nsec)*1e-9;
    tt3.col = col.tt3;
    tt3.name = name3;
end

cam_yolo.sens_stamp(cam_yolo.sens_stamp < 0) = NaN;

sensors = { ...
    struct('s', lid_clust, 'col', col.lidar,   'name', 'lidar'), ...
    struct('s', lid_pp,    'col', col.pp,      'name', 'pointpillars'), ...
    struct('s', rad_clust, 'col', col.radar,   'name', 'radar'), ...
    % struct('s', cam_yolo,  'col', col.camera,  'name', 'camera'), ...
};

if any(~isnan(v2v.sens_stamp(:)))
    sensors{end+1} = struct('s', v2v, 'col', col.v2v, 'name', 'v2v');
end

if(use_ref || use_sim_ref)
    errors = process_states(gt, tt, err_thr, err_stats);
    if(compare)
        errors2 = process_states(gt, tt2, err_thr, err_stats);
    end
    if(compare2)
        errors3 = process_states(gt, tt3, err_thr, err_stats);
    end
end


%% PLOTTING

% info;
%detections;
% latency;
%state_map;
%state_cog;
%range;
%speed_acc;
%P_diag;
%jerk_activation;
%map;
%covariance;
%error_analysis;
% imm;
% dataset_analysis;
%jerk_derivation;
%ff_jerk_flag;
%ramp;
%latency_delay;
%outlier_analysis;
%rts_sens_analysis
rts

link_axes();

    close all; clc;
clearvars -except colors log log_2 log_3 log_ref trajDatabase

link_axes_mode = 'all'; % 'none', 'figure', or 'all'

use_ref     = false;
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
name1 = 'new';

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
    name2 = 'old';
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
    name3 = 'old';
end

% load log ref
if(use_ref)
    if  (~exist('log_ref','var') || isempty(log_ref))
        [file,path] = uigetfile(fullfile(opp_dir,'*.mat'),'Load ground truth');
        tmp = load(fullfile(path,file));
        log_ref = tmp.out;
        clearvars tmp;
        gt = tmp.out;
    end
end
if(~exist('log_ref','var'))
    log_ref = [];
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% NAMING
graphics_options;
col.lidar   = colors.green{2};
col.radar   = [77 190 238] / 255;
col.conly  = colors.yellow{2};
col.cenh  = colors.orange{2};
col.conly2  = [1 0 1]; % magenta
col.cenh2   = [0.5 0 0.5]; % purple
col.ref     = colors.black;
sz=3; % Marker size
f=1;
c = 0;
x_lim = [0 inf];

%% PARSING

[lid_clust, rad_clust, cam_yolo, lid_pp, v2v] = load_perception(log);
cam_yolo.sens_stamp(cam_yolo.sens_stamp < 0) = NaN;

[cam_only, cam_enh] = splitCamYolo(cam_yolo);

if(use_ref || use_sim_ref); gt = load_ref(log, use_sim_ref, use_ref, log_ref); end
if(compare) 
    [lid_clust2, rad_clust2, cam_yolo2, lid_pp2, v2v2] = load_perception(log_2);
    cam_yolo2.sens_stamp(cam_yolo2.sens_stamp < 0) = NaN;
    cam_yolo2.stamp = cam_yolo2.stamp + double(log_2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    lid_clust2.stamp = lid_clust2.stamp + double(log_2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    rad_clust2.stamp = rad_clust2.stamp + double(log_2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    lid_pp2.stamp = lid_pp2.stamp + double(log_2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    [cam_only2, cam_enh2] = splitCamYolo(cam_yolo2);
end

sensors = { ...
    struct('s', cam_only,  'col', col.conly,  'name', 'cameraOnly'), ...
    struct('s', cam_only2,  'col', col.conly2,  'name', 'cameraOnly2'), ...
    struct('s', cam_enh,  'col', col.cenh,  'name', 'cameraEnh'), ...
    struct('s', cam_enh2,  'col', col.cenh2,  'name', 'cameraEnh'), ...
};


% Disable TT plots from figures
tt_exists = exist('tt', 'var');
tt2_exists=false; tt3_exists=false;
if (compare); tt2_exists = exist('tt2', 'var'); end
if (compare2); tt3_exists = exist('tt3', 'var'); end

% if(use_ref || use_sim_ref)
%     errors = process_states(gt, tt, err_thr, err_stats);
%     if(compare)
%         errors2 = process_states(gt, tt2, err_thr, err_stats);
%     end
%     if(compare2)
%         errors3 = process_states(gt, tt3, err_thr, err_stats);
%     end
% end


%% PLOTTING

% info;
% detections;
latency;
state_map;
state_cog;
range;
map;

% covariance;
% error_analysis;
% imm;
% dataset_analysis;

link_axes();

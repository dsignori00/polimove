%% Create Standard Mat
% save the opponent most useful data in a standard mat struct

% DATA:                     % DESCRIPTION:

% gps                       % gps used to extract the data
% primary_antenna           % primary antenna
% bag_timestamp             % msg timestamp
% timestamp                 % timestamp in epoch format (nanoseconds)
% header_stamp              % header stamp from driver
% latitude                  % latitude
% longitude                 % longitude
% bag_avg_frequency         % average publishing frequency
% x_map                     % x in global reference frame (ENU)
% y_map                     % y in global reference frame (ENU)
% z_map                     % z in global reference frame (ENU)
% yaw_map                   % heading in global reference frame 
% timestamp_diff            % difference between ego and opp timestamp
% x_rel                     % x in car reference frame 
% y_rel                     % y in car reference frame 
% z_rel                     % z in car reference frame 
% yaw_rel                   % relative yaw between ego and opp car 
% speed                     % opp speed
% ax                        % longitudinal acceleration
% virtual_speed             % true if speed calculated from gps pos measures
% virtual_acc               % true if acceleration is virtual (i.e. not measured)
% virtual_yawrate           % true if yawrate calculated from heading measures
% rho                       % distance from opponent (range)
% rho_dot                   % range rate (relative speed)
% clos_idx                  % traj server closest idx
% lap                       % lap counter
% ref_offset                % reference point offset to the vehicle center convention [m]


%% Load Data
clc; close all; clearvars -except ego v2v closest_idxs opp_idxs file_name out.clos_idx
proj = currentProject;

% load ego log
if (~exist('ego','var'))
    [file,normal_path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load ego mat');
    tmp = load(fullfile(normal_path,file));
    ego = tmp.log;
    clearvars tmp;
end

% load v2v estimation file
if (~exist('v2v','var'))
    [file,normal_path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load v2v mat');
    tmp = load(fullfile(normal_path,file));
    v2v = tmp.log;
    clearvars tmp;
end

%% Ego
ego_bag_timestamp = (ego.estimation.bag_stamp)*10^9+double(ego.time_offset_nsec);
ego_timestamp = (ego.estimation.stamp__tot)*10^9+double(ego.time_offset_nsec);
ego_x_map = ego.estimation.x_cog;
ego_y_map = ego.estimation.y_cog;
ego_z_map = ego.estimation.z_cog;
ego_yaw_map = ego.estimation.heading;
ego_roll = ego.estimation.roll;
ego_pitch = ego.estimation.pitch;
ego_speed = ego.estimation.vx;
ego_sz = length(ego_x_map);

% check if ros time is synchronized with gps time
gps_bag_stamp = ego.vectornav__raw__gps.bag_stamp;
if (gps_bag_stamp(1)>10)
    error("Ego ros time not sync to gps!")
end

%% Fill out log

out.bag_timestamp = (v2v.estimation.bag_stamp)*10^9+double(v2v.time_offset_nsec);
out.timestamp = (v2v.estimation.stamp__tot)*10^9+double(v2v.time_offset_nsec);
out.x_map = v2v.estimation.x_cog;
out.y_map = v2v.estimation.y_cog;
out.z_map = v2v.estimation.z_cog;
out.yaw_map = v2v.estimation.heading;
out.roll = ego.estimation.roll;
out.pitch = ego.estimation.pitch;
out.speed = ego.estimation.vx;
out.ax = v2v.ax;
out.sz = length(out.x_map);
out.gps = NaN;
out.primary_antenna = NaN;
out.header_stamp = NaN(out.sz,1);
out.latitude = NaN(out.sz,1);
out.longitude = NaN(out.sz,1);
out.virtual_speed = false;
out.virtual_acc = false;
out.virtual_yawrate = false;
out.ref_offset = 0.0;

% average frequency 
diff = out.timestamp(2:length(out.timestamp))-out.timestamp(1:length(out.timestamp)-1);
freq = 1./diff;
avg_freq = mean(freq)*10^9;
out.bag_avg_freq = avg_freq;

% check if ros time is synchronized with gps time
gps_bag_stamp = v2v.vectornav__raw__gps.bag_stamp;
if (gps_bag_stamp(1)>10)
    error("V2V ros time not sync to gps!")
end

%% Opponent and ego sync

if(~exist('closest_idxs','var') || ~exist('opp_idxs','var'))
    [closest_idxs, opp_idxs] = find_closest_stamp(out.timestamp,ego_timestamp);
end

time_diff_nsec = NaN(opp_sz,1); 
time_diff_nsec(opp_idxs) = ego_timestamp(closest_idxs) - out.timestamp(opp_idxs);
out.timestamp_diff = time_diff_nsec;

%% Assign closest idx and lap

if (~exist('out.clos_idx','var'))
    opp_pos = [out.x_map, out.y_map];
    [~, opp_idx] = get_heading(opp_pos, trajDatabase, 10);
    out.clos_idx = opp_idx;
    out.lap = assign_lap(opp_idx);
end

%% Relative metrics computation

dx = out.x_map(opp_idxs) - ego_x_map(closest_idxs);
dy = out.y_map(opp_idxs) - ego_y_map(closest_idxs);
dz = out.z_map(opp_idxs) - ego_z_map(closest_idxs);
yaw_rel = out.yaw_map(opp_idxs)-ego_yaw_map(closest_idxs);

x_rel = NaN(size(opp_idxs));
y_rel = NaN(size(opp_idxs));

for i = 1:length(opp_idxs)
    psi = ego_yaw_map(closest_idxs(i));

    % 2D rotation matrix
    R_yaw = [cos(psi), -sin(psi);
             sin(psi),  cos(psi)];

    % translation vector in XY
    translation_xy = [dx(i); dy(i)];
    opp_rel_xy = R_yaw' * translation_xy;

    x_rel(i) = opp_rel_xy(1);
    y_rel(i) = opp_rel_xy(2);
    
end

tot_x_rel = NaN(opp_sz,1);
tot_y_rel = NaN(opp_sz,1);
tot_z_rel = NaN(opp_sz,1);
tot_yaw_rel = NaN(opp_sz,1);

tot_x_rel(opp_idxs) = x_rel;
tot_y_rel(opp_idxs) = y_rel;
tot_z_rel(opp_idxs) = dz;         
tot_yaw_rel(opp_idxs) = yaw_rel;

out.x_rel = tot_x_rel;
out.y_rel = tot_y_rel;
out.z_rel = tot_z_rel;
out.yaw_rel = tot_yaw_rel;

%% Range and Range Rate

range = NaN(opp_sz,1);
r = sqrt(dx.^2 + dy.^2);
range(opp_idxs) = r;
out.rho = range;

beta = atan2(dy, dx);
v_rel_x = out.speed(opp_idxs).*cos(out.yaw_map(opp_idxs)) - ego_speed(closest_idxs).*cos(ego_yaw_map(closest_idxs));
v_rel_y = out.speed(opp_idxs).*sin(out.yaw_map(opp_idxs)) - ego_speed(closest_idxs).*sin(ego_yaw_map(closest_idxs));
valid_rho_dot = (dx .* v_rel_x + dy .* v_rel_y) ./ r;

rho_dot = NaN(opp_sz,1);
rho_dot(opp_idxs) = valid_rho_dot;
out.rho_dot = rho_dot;


%% Save the output struct
output_path = fullfile(proj.RootFolder, 'src', 'perception', 'opponent_gps', 'mat');

try
    output_file = fullfile(output_path, file_name + ".mat");
    save(output_file, 'out', '-v7.3');
    fprintf("File salvato correttamente: %s\n", output_file);
catch e
    warning("WARNING: Could not save files");
    warning("Error type:  " + e.message);
end

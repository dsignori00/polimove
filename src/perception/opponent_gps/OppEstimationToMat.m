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
% virtual_psidot            % true if psidot calculated from heading measures
% rho                       % distance from opponent (range)
% rho_dot                   % range rate (relative speed)
% clos_idx                  % traj server closest idx
% lap                       % lap counter

% NOTE: for UTM conversion, install following add-on
% https://github.com/geographiclib/geographiclib-octave

%% Load Data
clc; close all; clearvars -except ego opp_log closest_idxs opp_idxs file_name out.clos_idx
proj = currentProject;


% load ego log
if (~exist('ego','var'))
    [file,normal_path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load ego mat');
    tmp = load(fullfile(normal_path,file));
    ego = tmp.log;
    clearvars tmp;
end

% load opponent estimation file
if  (~exist('opp_log','var'))
    opp_data_folder = fullfile(proj.RootFolder, 'src', 'perception', 'opponent_gps', 'opp_data', '*.csv');
    [file,normal_path] = uigetfile(opp_data_folder,'Load opp localization');
    file = dir(fullfile(normal_path,file));
    files_list = string(file.name);

    %%Create complete opponent mat
    T = struct();
    i = 1;
    while i <= numel(files_list)
        try
            filename = files_list{i};
            [~, file_name, ~] = fileparts(filename);
    
            if ~isfield(T, file_name)
                fprintf("Loading: %s\n", filename);
                T = importCsv(filename);
            end
    
            i = i + 1;
        catch e
            warning("WARNING: Could not parse topic: %s", file_name);
            warning("Error type: " + e.message);
            files_list(i) = []; % remove bad entry
        end
    end

    % Remove file-name prefix from any columns
    vars = T.Properties.VariableNames;
    prefix = file_name + "_";
    for k = 1:numel(vars)
        if startsWith(vars{k}, prefix)
            vars{k} = erase(vars{k}, prefix);  
        end
    end
    T.Properties.VariableNames = vars;

    % Store all data in just 1 struct
    opp_log = table2struct(T, "ToScalar", true);
end

%% Fill out log

%Select opponent and gps
opp_id = -1;
lista_opponents = [0,2,3,6,33,71];
while ~ismember(opp_id, lista_opponents)
    disp( "Choose opponent:" + newline + ...
        " 3: Kinetiz " + newline + ...
        " 6: Unimore " + newline + ...
        "33: Tum" + newline + ...
        "71: Tii " + newline + ...
        " 2: Purdue " + newline + ...
        " 0: Quit");
    opp_id = input("Choose opponent identifier: ");
end


switch(opp_id)
    case 2 
        ref_sys = "lla";
        opp_lat0 = 36.58733835;
        opp_lon0 = -121.75566957;
        opp_alt0 = -2.35619;
        out.timestamp = opp_log.stamp*10^9;
        out.x_map = opp_log.x_cog;
        out.y_map = opp_log.y_cog;
        out.yaw_map = unwrap(opp_log.heading);
        out.speed = opp_log.vx;
        out.ax = opp_log.ax;

    case 3
        ref_sys = "lla";
        opp_lat0 = 44.344351;
        opp_lon0 = 11.714010;
        opp_alt0 = 0.0;
        out.timestamp = opp_log.timestamp*10^9;
        out.x_map = opp_log.x_m;
        out.y_map = opp_log.y_m;
        out.yaw_map = unwrap(opp_log.yaw_rad);
        out.speed = opp_log.vx_mps;
        out.ax = opp_log.ax_mps2;

    case 6
        ref_sys = "utm";
        opp_lat0 = 36.586455;
        opp_lon0 = -121.756645;
        opp_alt0 = 0.0;
        out.timestamp = opp_log.stamp;
        out.x_map = opp_log.x;
        out.y_map = opp_log.y;
        out.yaw_map = unwrap(opp_log.yaw);
        out.speed = opp_log.vx;
        out.ax = opp_log.ax;

    case 33
        ref_sys = "lla";
        opp_lat0 = 44.342534100;
        opp_lon0 = 11.711892000;
        opp_alt0 = -39.992;
        out.timestamp = opp_log.stamp*10^9;
        out.x_map = opp_log.x_cog;
        out.y_map = opp_log.y_cog;
        out.yaw_map = unwrap(opp_log.heading());
        out.speed = opp_log.vx;
        out.ax = opp_log.ax;

    case 71
        ref_sys = "lla";
        opp_lat0 = 24.46992202098782;
        opp_lon0 = 54.60522506805341;
        opp_alt0 = 0.0;
        out.timestamp = opp_log.timestamp*10^9;
        out.x_map = opp_log.position_x;
        out.y_map = opp_log.position_y;
        out.yaw_map = unwrap(opp_log.yaw);
        out.speed = opp_log.vel_x;
        out.ax = opp_log.ax;

end

%Select the track
track_list = [0,1,2,3,4,5,6,7];
track_id = -1;
while ~ismember(track_id, track_list)
    disp( "Choose track:" + newline + ...
          " 1: KS " + newline + ...
          " 2: IMS " + newline + ...
          " 3: LVMS " + newline + ...
          " 4: YasMarina " + newline + ...
          " 5: YasNorth " + newline + ...
          " 6: Imola " + newline + ...
          " 7: LagunaSeca " + newline + ...
          " 0: Quit");
    track_id = input("Insert track identifier: ");
end

% Geodetic to Enu
wgs84 = wgs84Ellipsoid;
switch (track_id) 
    case 1 
        % KS
        lat0    = 38.711552404047440;
        lon0    = -84.916952255229230;
        alt0    = 182.9;
        load("Ks.mat")
    case 2
        % IMS
        lat0    =  39.793145808368780;	
        lon0    = -86.236780583175840;
        alt0    =  221.8500178;
        load("Ims.mat")
    case 3
        % LVMS
        lat0    =  36.272904305728540;
        lon0    = -115.0110198639284+1e-5;
        alt0    =  594.6250982;
        load("Lvms.mat")
    case 4
        % YasMarina
        lat0 = 24.470253250335873;
        lon0 = 54.605170726971520;
        alt0 = 182.9;
        load("YasMarina.mat")
     case 5
        % YasNorth
        lat0 = 24.470253250335873 - 2.3448e-5;
        lon0 = 54.605170726971520;
        alt0 = 182.9;
        load("YasNorth.mat")
     case 6
        % Imola
        lat0 = 44.34266668085731;
        lon0 = 11.71339346495961;
        alt0 = 40.0;
        load("Imola.mat")
     case 7
         lat0 = 36.586062395673760 + 1.0e-5;
         lon0 = -121.7562012288262;
         alt0 = 231.0461153351115;
         load("LagunaSeca.mat")
    case 0
        error("Quit");
    otherwise
        error("Error in track selection");  
end

if strcmp(ref_sys, "lla")
    [x0,y0,~] = geodetic2enu(opp_lat0,opp_lon0,opp_alt0,lat0,lon0,alt0,wgs84);
    out.x_map = out.x_map + x0;
    out.y_map = out.y_map + y0;
elseif strcmp(ref_sys, "utm")
    [originEasting, originNorthing, zone, isNorth] = utmups_fwd(opp_lat0, opp_lon0);
    easting = out.x_map + originEasting;
    northing = out.y_map + originNorthing;
    [latitude, longitude, meridianConvergence] = utmups_inv(easting, northing, zone, isNorth);
    [out.x_map, out.y_map, ~] = geodetic2enu(latitude, longitude, zeros(size(latitude)), lat0, lon0, alt0, wgs84);
    out.yaw_map = unwrap(out.yaw_map - deg2rad(meridianConvergence));
else
    disp("Unrecognized reference system")
end


opp_sz = length(out.timestamp);
out.gps = NaN;
out.primary_antenna = NaN;
out.bag_timestamp = NaN(opp_sz,1);
out.header_stamp = NaN(opp_sz,1);
out.latitude = NaN(opp_sz,1);
out.longitude = NaN(opp_sz,1);
out.z_map = NaN(opp_sz,1);
out.z_rel = NaN(opp_sz,1);
out.virtual_speed = false;

% average frequency 
diff = out.timestamp(2:length(out.timestamp))-out.timestamp(1:length(out.timestamp)-1);
freq = 1./diff;
avg_freq = mean(freq)*10^9;
out.bag_avg_freq = avg_freq;

framelen = floor(avg_freq / 2);
if(mod(framelen,2)==0)
    framelen = framelen + 1;
end 

% acceleration
if opp_id == 3 || opp_id == 33
    imu_cutoff_freq = 5;
    imu_filter_order = 2;

    if ~isfinite(avg_freq) || avg_freq <= 2 * imu_cutoff_freq
        error("The IMU sample frequency must be greater than twice the cutoff frequency (%.1f Hz).", imu_cutoff_freq);
    end

    [imu_filter_b, imu_filter_a] = butter(imu_filter_order, imu_cutoff_freq / (avg_freq / 2), "low");
    out.ax = filtfilt(imu_filter_b, imu_filter_a, out.ax);
end

if ~isfield(out,'ax') || all(isnan(out.ax))
    out.ax = sgolayfilt(gradient(out.speed(:)) ./ gradient(out.timestamp(:))*10^9, 3, framelen);
    out.virtual_acc = true;
end

% yaw rate
if ~isfield(out,'yaw_rate') || all(isnan(out.yaw_rate))
    out.yaw_rate = sgolayfilt(gradient(out.yaw_map(:)) ./ gradient(out.timestamp(:))*10^9, 3, framelen);
    out.virtual_yawrate = true;
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
    error("Ros time not sync to gps!")
end

%% Opponent and ego sync

if(~exist('closest_idxs','var') || ~exist('opp_idxs','var'))
    [closest_idxs, opp_idxs] = find_closest_stamp(out.timestamp,ego_timestamp);
end
ego_bag_timestamp = ego_bag_timestamp(closest_idxs);

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
z_rel = NaN(size(opp_idxs));

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
tot_z_rel(opp_idxs) = z_rel;         
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

% TO DO:
% - rho_dot estimation (velocità radiale)

%% Create Standard Mat
% save the opponent most useful data in a standard mat struct

% DATA:                     % DESCRIPTION:

% gps                       % gps used to extract the data
% primary_antenna           % primary antenna
% bag_timestamp             % msg timestamp
% timestamp                 % timestamp in epoch format (nanoseconds)
% header_stamp              % header stamp from driver
% latitude                  % latitude
% longitude                 % longiude
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
% calculated_speed          % true if speed calculated from gps pos measures
% calculated_acceleration   % true if acceleration derived from speed
% rho                       % distance from opponent (range)
% rho_dot                   % range rate (relative speed)
% clos_idx                  % traj server closest idx
% lap                       % lap counter

close all
clearvars -except ego opp_dir_path 

%% Paths

proj = currentProject();
addpath(fullfile(proj.RootFolder,'src','perception','utils'));
opp_dir_path = "opp_data/";
addpath(opp_dir_path);

%% Input

% load opponent gps file
files = dir(fullfile(opp_dir_path, '*.csv'));
files_list = string({files.name});


% load ego log
if (~exist('ego','var'))
    [file,normal_path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load ego mat');
    tmp = load(fullfile(normal_path,file));
    ego = tmp.log;
    clearvars tmp;
end

%Select the track
track_list = [0,1,2,3,4];
track_id = -1;
while ~ismember(track_id, track_list)
    disp( "Choose track:" + newline + ...
          " 1: KS " + newline + ...
          " 2: IMS " + newline + ...
          " 3: LVMS " + newline + ...
          " 4: YasMarina " + newline + ...
          " 0: Quit");
    track_id = input("Insert track identifier: ");
end

% Assign IAC or A2RL based on the selection
a2rl_tracks = 4;
if ismember(track_id, a2rl_tracks)
    disp('Track selected. A2RL will be used.');
    competition = 'A2RL';
else
    disp('Track selected. IAC will be used.');
    competition = 'IAC';
end

%Select opponent and gps
opp_id = -1;
gps_id = -1;

if competition == "IAC"
    lista_opponents = [0,3,6,9];
    lista_gps = [0,1,2,3];
    while ~ismember(opp_id, lista_opponents)
        disp( "Choose opponent:" + newline + ...
              " 3: Tum " + newline + ...
              " 6: Unimore " + newline + ...
              " 9: Cavalier " + newline + ...
              " 0: Quit");
        opp_id = input("Choose opponent identifier: ");
    end
    while ~ismember(gps_id, lista_gps)
        disp( "Choose gps:" + newline + ...
              " 1: Vectornav " + newline + ...
              " 2: Novatel top " + newline + ...
              " 3: Novatel bot" + newline + ...
              " 0: Quit");
        gps_id = input("Choose gps identifier: ");
    end
elseif competition == "A2RL"
    lista_opponents = [0,3,6];
    while ~ismember(opp_id, lista_opponents)
        disp( "Choose opponent:" + newline + ...
              " 3: Tum " + newline + ...
              " 6: Unimore " + newline + ...
              " 0: Quit");
        opp_id = input("Choose opponent identifier: ");
    end
    gps_id = 1;
end

%% Create complete opponent mat

for i = 1:length(files_list)
    try       
        filename = files_list(i);
        file_name = sprintf("%s", erase(filename, ".csv"));
        if  (~exist(filename,'var'))
            fprintf("Loading: %s\n", filename);
            eval_string_1 = file_name + " = importCsv('" + filename + "');";
            eval(eval_string_1);
        
            eval_string_2 = "fieldnames(" + file_name + ");";
            fields = eval(eval_string_2);
        end
    catch e
            warning("WARNING: Could not parse topic: " + file_name);
            warning("Error type:  " + e.message);
            files_list = files_list([1:i-1,i+1:end]);
    end
end

% Save all data in just 1 struct
[~, output_filename, ~] = fileparts(opp_dir_path);
output_path = fullfile(opp_dir_path, output_filename + "_raw.mat");
fprintf("\nSaving data to: %s\n", output_path);

for i = 1:length(files_list)
    filename = files_list(i);
    topic_name = sprintf("%s", erase(filename, ".csv"));

    isCellCol = varfun(@iscell, eval(topic_name), 'OutputFormat', 'uniform');
    
    eval_string_9 = topic_name + "(:, isCellCol) = [];";
    eval(eval_string_9);

    eval_string_10 = "fields = fieldnames(" + topic_name + ");";
    eval(eval_string_10);

    for j = 1:length(fields)

        eval_string_value = topic_name + "." + fields{j};
        field_value = eval(eval_string_value);  

        if  (~iscell(field_value))             
            eval_string_11 = "opp_log.(fields{j}) = " + eval_string_value + ";";
            eval(eval_string_11);
        end  
    end
end

try
    opp_log = rmfield(opp_log, {'Properties','Variables'});
    save(output_path, 'opp_log', '-v7.3');
catch e
    warning("WARNING: Could not save files");
    warning("Error type:  " + e.message);
end



%% Ego State

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

%% Ego sync with Gps time
% check if ros time is synchronized with gps time
gps_bag_stamp = ego.vectornav__raw__gps.bag_stamp;
if (gps_bag_stamp(1)>10)
    error("Ros time not sync to gps!")
end

%% Geodetic to ENU

% Get timestamp and Geodetic coordinates

% NB undulation already included in novatel measures, not in vectornav ones
try
    undulation = ego.novatel_bottom__bestgnsspos.undulation(1);
catch
    undulation = 0;
end

switch opp_id
    case 6
        if(gps_id==1)
        log.gps             = "vectornav";
        if competition == "IAC"
            log.primary_antenna = "front";
        elseif competition == "A2RL"
            log.primary_antenna = "top";
        end
        log.bag_timestamp   = opp_log.vectornav_gps_a_stamp;
        log.header          = opp_log.vectornav_gps_a_head_stamp;
        log.timestamp       = opp_log.vectornav_gps_a_utcStamp;     
        log.latitude        = opp_log.vectornav_gps_a_latitude;
        log.longitude       = opp_log.vectornav_gps_a_longitude;
        log.altitude        = opp_log.vectornav_gps_a_altitude-undulation;
        raw_speed.ve        = opp_log.vectornav_gps_a_speed_value0;
        raw_speed.vn        = opp_log.vectornav_gps_a_speed_value1;
        raw_speed.vz        = opp_log.vectornav_gps_a_speed_value2;

        elseif(gps_id==2)         
        log.gps             = "novatel_top";
        log.primary_antenna = "front";
        log.bag_timestamp   = opp_log.novatel_btm_gps_stamp;
        log.header          = opp_log.novatel_top_gps_head_stamp;
        log.timestamp       = opp_log.novatel_top_gps_utcStamp;
        log.latitude        = opp_log.novatel_top_gps_latitude;
        log.longitude       = opp_log.novatel_top_gps_longitude;
        log.altitude        = opp_log.novatel_top_gps_altitude;
        raw_speed.ve        = opp_log.novatel_top_gps_speed_value0;
        raw_speed.vn        = opp_log.novatel_top_gps_speed_value1;
        raw_speed.vz        = opp_log.novatel_top_gps_speed_value2;

        elseif(gps_id==3)
        log.gps             = "novatel_btm";
        log.primary_antenna = "right";
        log.bag_timestamp   = opp_log.novatel_btm_gps_stamp;
        log.header          = opp_log.novatel_btm_gps_head_stamp;
        log.timestamp       = opp_log.novatel_btm_gps_utcStamp;
        log.latitude        = opp_log.novatel_btm_gps_latitude;
        log.longitude       = opp_log.novatel_btm_gps_longitude;
        log.altitude        = opp_log.novatel_btm_gps_altitude;
        raw_speed.ve        = opp_log.novatel_btm_gps_speed_value0;
        raw_speed.vn        = opp_log.novatel_btm_gps_speed_value1;
        raw_speed.vz        = opp_log.novatel_btm_gps_speed_value2;
        end

        log.timestamp = utcToGpsEpoch(log.timestamp);
    
    case 9
        if(gps_id==1)
        error("VectorNav data not present!")

        elseif(gps_id==2)
        log.gps             = "novatel_top";
        log.primary_antenna = "front";
        log.bag_timestamp   = opp_log.novatel_top__bestpos__bag_timestamp;
        log.header_stamp    = opp_log.novatel_top__bestpos__header__stamp__sec*10^9+opp_log.novatel_top__bestpos__header__stamp__nanosec;
        log.latitude        = opp_log.novatel_top__bestpos__lat;
        log.longitude       = opp_log.novatel_top__bestpos__lon;
        log.altitude        = opp_log.novatel_top__bestpos__hgt;
        week_num            = opp_log.novatel_top__bestpos__nov_header__gps_week_number;
        week_msec           = opp_log.novatel_top__bestpos__nov_header__gps_week_milliseconds;
   

        elseif(gps_id==3)
        log.gps             = "novatel_btm";
        log.primary_antenna = "right";
        log.bag_timestamp   = opp_log.novatel_bottom__bestpos__bag_timestamp;
        log.header          = opp_log.novatel_bottom__bestpos__header__stamp__sec*10^9+opp_log.novatel_bottom__bestpos__header__stamp__nanosec;
        log.latitude        = opp_log.novatel_bottom__bestpos__lat;
        log.longitude       = opp_log.novatel_bottom__bestpos__lon;
        log.altitude        = opp_log.novatel_bottom__bestpos__hgt;
        week_num            = opp_log.novatel_bottom__bestpos__nov_header__gps_week_number;
        week_msec           = opp_log.novatel_bottom__bestpos__nov_header__gps_week_milliseconds;
        end
        
        log.timestamp = towToUtc(week_num,week_msec);
        log.timestamp = utcToGpsEpoch(log.timestamp);

    case 0
        error("Quit");
    otherwise
        error('Error in opponent selection')
end

% sort opp data wrt timestamp
[log.timestamp, sorted_idx] = sort(log.timestamp);

log.bag_timestamp   = log.bag_timestamp(sorted_idx);
log.latitude        = log.latitude(sorted_idx);
log.longitude       = log.longitude(sorted_idx);
log.altitude        = log.altitude(sorted_idx);

% average frequency 
diff = log.timestamp(2:length(log.timestamp))-log.timestamp(1:length(log.timestamp)-1);
freq = 1./diff;
avg_freq = mean(freq)*10^9;
log.bag_avg_freq = avg_freq;

% Geodetic to Enu
wgs84 = wgs84Ellipsoid;
switch (track_id) 
    case 1 
        % KS
        lat0    = 38.711552404047440;
        lon0    = -84.916952255229230;
        alt0    = 182.9;
        load("Ks.mat");
    case 2
        % IMS
        lat0    =  39.793145808368780;	
        lon0    = -86.236780583175840;
        alt0    =  221.8500178;
        load("Ims.mat");
    case 3
        % LVMS
        lat0    =  36.272904305728540;
        lon0    = -115.0110198639284+1e-5;
        alt0    =  594.6250982;
        load("Lvms.mat");
    case 4
        % YasMarina
        lat0 = 24.470253250335873;
        lon0 = 54.605170726971520;
        alt0 = 182.9;
        load("YasMarina.mat");
    case 0
        error("Quit");
    otherwise
        error("Error in track selection");  
end

[x_map,y_map,~] = geodetic2enu(log.latitude,log.longitude,log.altitude,lat0,lon0,alt0,wgs84);
log.x_map = x_map;
log.y_map = y_map;
opp_sz = length(x_map);

%% Opponent and ego sync

%[log.timestamp] = latency_removal(log.timestamp,log.gps);
[closest_idxs, opp_idxs] = find_closest_stamp(log.timestamp,ego_timestamp);
ego_bag_timestamp = ego_bag_timestamp(closest_idxs);

time_diff_nsec = NaN(opp_sz,1); 
time_diff_nsec(opp_idxs) = ego_timestamp(closest_idxs) - log.timestamp(opp_idxs);
log.timestamp_diff = time_diff_nsec;
log.header_timestamp_ego =  ego_timestamp(closest_idxs);


%% Orientation

pitch = zeros(opp_sz,1);
roll = zeros(opp_sz,1);
yaw = zeros(opp_sz,1);
z_map = zeros(opp_sz,1);


for j=1:length(x_map)
    x = x_map(j);
    y = y_map(j);
    min_dist = inf;
    traj = 0;
    for i=1:length(trajDatabase)
        [dist, idx] = compute_dist(x,y,trajDatabase(i).X,trajDatabase(i).Y);
        if(dist<min_dist)
            min_dist = dist;
            clos_idx = idx;
            traj = i;
        end
    end
    pitch(j,1) = trajDatabase(traj).Slope(clos_idx);
    roll(j,1) = trajDatabase(traj).Banking(clos_idx);
    yaw(j,1) = trajDatabase(traj).Heading(clos_idx);
    z_map(j,1) = trajDatabase(traj).Z(clos_idx) + 0.3;
   
end
yaw = deg2rad(UnwrapPi(rad2deg(yaw)));
yaw = correct_yaw(x_map,y_map,log.timestamp,log.bag_avg_freq,yaw);

log.z_map = z_map;
log.roll = roll;
log.pitch = pitch;
log.yaw_map = yaw;


%% Opponent speed and acceleration

if  (exist('raw_speed','var'))
    speed = sqrt(raw_speed.ve.^2+raw_speed.vn.^2);
    log.computed_speed = false;
else
    speed = compute_opp_speed(x_map,y_map,log.timestamp,log.bag_avg_freq);
    log.compute_speed = true;
end
log.speed = speed;

acc = NaN(opp_sz,1);
log.calculated_acceleration = false;
if ~computed_speed
    acc = sgolayfilt(gradient(speed(:)) ./ gradient(log.timestamp(:))*10^9, 3, 21);
    log.ax = acc;
    log.calculated_acceleration = true;
end



%% Compensate for antenna offset
% Adjust for the center_of_gravity if we know the offset (from antenna to center)

% IAC
if competition == "IAC"
    switch (log.primary_antenna)
        case "front"
            % Primary front  BOT_GPS_DXYZ = {1.405, 0.0, 0.483};
            x_offset = -1.405;
            y_offset = 0;
            z_offset = -0.483;
        case "top"
            % Primary top  VN_GPS2_DXYZ  = {-0.5, 0.0, 1.031}
            x_offset = 0.5;
            y_offset = 0;
            z_offset = - 1.031;
        case "right"
            % Primary lat dx TOP_GPS_DXYZ = {0.284, -0.497, 0.464};
            x_offset = -0.284;
            y_offset = 0.497;
            z_offset = -0.464; 
        case "left"
            % Primary lat sx TOP_GPS2_DXYZ = {0.284, 0.497, 0.464};
            x_offset = - 0.284;
            y_offset = - 0.497;
            z_offset = - 0.464; 
        otherwise
            error("Error in Antenna selection");

    end
elseif competition == "A2RL"
    % A2RL
    switch (log.primary_antenna)
        case "front"
            % Primary front VN_GPS_DXYZ   = {1.699, 0.0, 0.527};    
            x_offset = -1.699;
            y_offset = 0;
            z_offset = -0.527;
        case "top"
            % Primary top  VN_GPS2_DXYZ  = {-0.23, 0.0, 1.045}
            x_offset = 0.23;
            y_offset = 0;
            z_offset = - 1.045;
        otherwise
            error("Error in Antenna selection");
    end
end

x_map_cog = NaN(opp_sz,1);
y_map_cog = NaN(opp_sz,1);

for i=1:opp_sz

    R = euler_to_rotation(roll(i),pitch(i),yaw(i));
    z_antenna = z_map(i) - (R(3,1)*x_offset+R(3,2)*y_offset+R(3,3)*z_offset);
    offset = [x_offset; y_offset; z_offset];
    t = [x_map(i); y_map(i); z_antenna];
    
    cog = R*offset+t;

    x_map_cog(i) = cog(1);
    y_map_cog(i) = cog(2);

end

log.x_map = x_map_cog;
log.y_map = y_map_cog;


%% Relative metrics computation

dx = x_map_cog(opp_idxs) - ego_x_map(closest_idxs);
dy = y_map_cog(opp_idxs) - ego_y_map(closest_idxs);
dz = z_map(opp_idxs) - ego_z_map(closest_idxs);
yaw_rel = yaw(opp_idxs)-ego_yaw_map(closest_idxs);

x_rel = NaN(size(opp_idxs));
y_rel = NaN(size(opp_idxs));
z_rel = NaN(size(opp_idxs));

for i = 1:length(opp_idxs)

    R = euler_to_rotation(ego_roll(closest_idxs(i)),ego_pitch(closest_idxs(i)),ego_yaw_map(closest_idxs(i)));
    translation = [dx(i);dy(i);dz(i)];
    opp_rel = R'*translation;

    if any(isnan(opp_rel))
        error('nan')
    end

    x_rel(i) = opp_rel(1);
    y_rel(i) = opp_rel(2);
    z_rel(i) = opp_rel(3);
end

tot_x_rel = NaN(opp_sz,1);
tot_y_rel = NaN(opp_sz,1);
tot_z_rel = NaN(opp_sz,1);
tot_yaw_rel = NaN(opp_sz,1);

tot_x_rel(opp_idxs) = x_rel;
tot_y_rel(opp_idxs) = y_rel;
tot_z_rel(opp_idxs) = z_rel;         
tot_yaw_rel(opp_idxs) = yaw_rel;

log.x_rel = tot_x_rel;
log.y_rel = tot_y_rel;
log.z_rel = tot_z_rel;
log.yaw_rel = tot_yaw_rel;


%% Range
range = NaN(opp_sz,1);
v = [dx;dy;dz];
r = norm(v);
range(opp_idxs) = r;
log.rho = range;

% rho dot
% dx = ego_x_map(closest_idxs)-x_map(opp_idxs);
% dy = ego_y_map(closest_idxs)-y_map(opp_idxs);
% beta = atan2(dy,dx);
% valid_rho_dot = (speed(opp_idxs).*cos(yaw(opp_idxs))-ego_speed(closest_idxs).*cos(ego_yaw_map(closest_idxs))).*cos(beta)+ ...
%                 (speed(opp_idxs).*cos(yaw(opp_idxs))-ego_speed(closest_idxs).*cos(ego_yaw_map(closest_idxs))).*sin(beta);
valid_rho_dot = speed(opp_idxs)-ego_speed(closest_idxs);

rho_dot = NaN(opp_sz,1);
rho_dot(opp_idxs) = valid_rho_dot;
log.rho_dot = rho_dot;

% some number of elements for each field
log.gps(1:opp_sz,1) = log.gps(1);
log.primary_antenna(1:opp_sz,1) = log.primary_antenna(1);
log.bag_avg_freq(1:opp_sz,1) = log.bag_avg_freq(1);
log.computed_speed(1:opp_sz,1) = log.computed_speed(1);

%%

try
    output_path = fullfile(opp_dir_path, output_filename + "_" +string(log.gps(1)) +".mat");
    fprintf("Saving processed data to: %s\n", output_path);
    save(output_path, 'log', '-v7.3');
catch e
    warning("WARNING: Could not save processed data");
    warning("Error type:  " + e.message);
end

fprintf("\nRemember to check if antenna's configuration has been changed!!!\n")
% clearvars -except ego opp_dir_path 

%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%  USEFUL FUNCTIONS  %%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function timestamp = latency_removal(timestamp,gps)
    for i=2:length(timestamp)
        ts = timestamp(i)-timestamp(i-1);
        if(strcmp(gps,'novatel_top') || strcmp(gps,'novatel_btm'))
             if((ts>6e+07) && ts<1e+09)
                timestamp(i) = timestamp(i-1)+5e+07;
             end
        elseif(strcmp(gps,'vectornav'))
             if((ts>2.18e+08 || ts<1.8e+08) && ts<1e+09)
                timestamp(i) = timestamp(i-1)+2e+08;
             end
        end
    end
end


function timestamp = latency_removal2(timestamp,gps)
    ts = timestamp(2:end)-timestamp(1:end-1);
    ts = [0; ts];
    for i=2:length(timestamp)
        if(strcmp(gps,'novatel_top') || strcmp(gps,'novatel_btm'))
             if(ts(i)<1e+09)
                timestamp(i) = timestamp(i-1)+5e+07;
             end
        elseif(strcmp(gps,'vectornav'))
             if(ts(i)<1e+09)
                timestamp(i) = timestamp(i-1)+2e+08;
             end
        end
    end
end

function yaw_out = correct_yaw(x_map,y_map, timestamp, avg_freq, yaw_in)
    
    
    time_th = 3/avg_freq*10^9;
    yaw_out = yaw_in;

    for i = 2:length(x_map)
        if (timestamp(i)-timestamp(i-1)<time_th)
            dx = x_map(i)-x_map(i-1); 
            dy = y_map(i)-y_map(i-1);
            ds = sqrt(dx^2+dy^2);
            dt = 1/avg_freq;
            v = ds/dt;
            if v>16
                yaw_out(i) = atan2(dy,dx);
            end
        end      
    end

end


function speed = compute_opp_speed(x_map,y_map, timestamp, avg_freq)
    
    time_th = 3/avg_freq*10^9;
    speed = NaN(length(x_map),1);
    inst_speed = NaN(length(x_map),1);
    for i = 2:length(x_map)
        dt = timestamp(i)-timestamp(i-1);
        if (dt<time_th)
            dx = x_map(i)-x_map(i-1); 
            dy = y_map(i)-y_map(i-1);
            ds = sqrt(dx^2+dy^2);
            dts = dt*10^-9;
            if (dt~=0 && ds/dts<90)
                inst_speed(i) = ds/dts; 
            end

            if i>8
                speed(i) = mean(inst_speed(i-8:i));
            else
                speed(i) = inst_speed(i);
            end
        end      
    end

end

function gps_time_ns = utcToGpsEpoch(utc_time_ns)
    % Constants
    GPS_UTC_DELTA_SEC = 315964800; 
    LEAP_SEC = 18;                 

    % Convert nanoseconds to seconds and remainder nanoseconds
    utc_sec = floor(utc_time_ns / 1e9);
    remainder_ns = mod(utc_time_ns, 1e9);

    % Calculate GPS time by adjusting for GPS-UTC offset and leap seconds
    gps_sec = utc_sec + GPS_UTC_DELTA_SEC - LEAP_SEC;

    % Combine back into nanoseconds
    gps_time_ns = gps_sec * 1e9 + remainder_ns;
end


function gps_time_ns = towToUtc(week,msec)
    
    WEEK2SEC = 7 * 24 * 60 * 60;

    gps_time_ms = week*WEEK2SEC*10^3+msec;
    gps_time_ns = gps_time_ms*10^6;
end
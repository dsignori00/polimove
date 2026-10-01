close all
clearvars -except log log2 trajDatabase gt ego ego2 rad1 rad2

ground_truth = false;
compare      = false;

NAME_1 = "Radar";
NAME_2 = "Radar - 2";

%#ok<*UNRCH>
%#ok<*INUSD>

%% Paths

proj = currentProject();
addpath(fullfile(proj.RootFolder,'src','perception','utils'));

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
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    load(fullfile(path,file));
end

% load log2
if (compare && ~exist('log2','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log 2');
    tmp = load(fullfile(path,file));
    log2 = tmp.log;
end

if(ground_truth)
    if (~exist('gt','var'))
        [file,path] = uigetfile(fullfile(get_gt_dir(),'*.mat'),'Load ground truth log');
        tmp = load(fullfile(path,file));
        gt = tmp.out;
    end
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% PLOT DATA

% style
set(0,'DefaultFigureWindowStyle','docked');
set(0,'DefaultTextInterpreter', 'none');
set(0,'DefaultLegendInterpreter', 'none');
set(0, 'DefaultLineLineWidth', 2);
x_lim = [0 inf];

%% NAMING
col.radar = '#0072BD';
col.radar2 = '#FF0000';
col.ref = '#000000';
sz=3; % Marker size
f=1;

% RADAR CLUSTERING DETECTIONS
rad1 = get_radar_fields(log);
if(compare); rad2 = get_radar_fields(log2); end


% BRIDGE POSITIONS
BRIDGE_POSITION.THRESHOLD = 5;

track = -1;
tracks = [0,1];
while ~ismember(track, tracks)
    disp( "Choose track:" + newline + ...
        " 1: Laguna Seca " + newline + ...
        " 0: No bridges");
    track = input("Choose track identifier: ");
end
switch track
    case 1
        BRIDGE_POSITION.X_MAP = [-65.571, -66.7, -61.9, 586.3, 591.4, 630.4, 451.6, 458.7, 190.6, 83.4, -63.4, -58.5, 86.2, 85.8, 183.5, 451.0, 452.9, 460.0, 623.7];
        BRIDGE_POSITION.Y_MAP = [1.693, 7.7, 4.8, -25.9, -26.3, -304.2, -650.1, -649.0, -534.1, -150.2, 14.8, 11.4, -151.5, -144.0, -532.2, -653.3, -656.4, -655.1, -303.9];
    otherwise
        BRIDGE_POSITION.X_MAP = [];
        BRIDGE_POSITION.Y_MAP = [];
        warning('No bridge positions defined for this database.');
end
%% PROCESSING

if(ground_truth); gt_timestamp = (gt.timestamp - double(log.time_offset_nsec))*10^-9; end

% compute heading
if (~exist('rad1.idx','var'))
    for i = 1:rad1.max_det
        opp_pos = [rad1.x_map(:,i), rad1.y_map(:,i)];
        [opp_yaw, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
        rad1.yaw_map(:,i) = opp_yaw;
        rad1.idx(:,i) = opp_idx;
    end
end
if(compare)
    if (~exist('rad2.idx','var'))
        for i = 1:rad2.max_det
            opp_pos = [rad2.x_map(:,i), rad2.y_map(:,i)];
            [opp_yaw, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
            rad2.yaw_map(:,i) = opp_yaw;
            rad2.idx(:,i) = opp_idx;
        end
    end
end

% match timestamp
ego.timestamp = (log.estimation.stamp__tot)*10^9+double(log.time_offset_nsec);
rad1_stamp = rad1.sens_stamp*10^9 + double(log.time_offset_nsec);

if (~exist('ego.clos_idx','var'))
    [ego.clos_idxs, ~] = find_closest_stamp(rad1_stamp,ego.timestamp);
end

ego.pos(:,1) = log.estimation.x_cog(ego.clos_idxs);
ego.pos(:,2) = log.estimation.y_cog(ego.clos_idxs);
ego.yaw = log.estimation.heading(ego.clos_idxs);
ego.v(:,1) = log.estimation.vx(ego.clos_idxs);
ego.v(:,2) = log.estimation.vy(ego.clos_idxs);

if(compare)
    rad2_epoch = rad2.sens_stamp*10^9 + double(log2.time_offset_nsec);
    if (~exist('ego2','var'))
        [ego2.clos_idxs, ~] = find_closest_stamp(rad2_epoch,ego.timestamp);
        if (isempty(ego2.clos_idxs))
            error('No timestamp matched between ego and opp.');
        end
    end
    ego2.pos(:,1) = log.estimation.x_cog(ego2.clos_idxs);
    ego2.pos(:,2) = log.estimation.y_cog(ego2.clos_idxs);
    ego2.yaw = log.estimation.heading(ego2.clos_idxs);
    ego2.v(:,1) = log.estimation.vx(ego2.clos_idxs);
    ego2.v(:,2) = log.estimation.vy(ego2.clos_idxs);
end

% compute opp speed
for i = 1:rad1.max_det
    opp_yaw = rad1.yaw_map(:,i);
    rad1.vx(:,i) = compensate_ego_speed(ego.pos, ego.v, ego.yaw, opp_yaw, ...
                        rad1.rho_dot(:,i), rad1.x_map(:,i), rad1.y_map(:,i));
end
rad1.range = sqrt(rad1.x_rel.^2 + rad1.y_rel.^2);

if(compare)
    for i = 1:max(rad2.max_det,1)
        opp_yaw = rad2.yaw_map(:,i);
        rad2.vx(:,i) = compensate_ego_speed(ego2.pos, ego2.v, ego2.yaw, opp_yaw, ...
                             rad2.rho_dot(:,i), rad2.x_map(:,i), rad2.y_map(:,i));
    end
    rad2_sens_stamp = rad2.sens_stamp + double(log2.time_offset_nsec)*10^-9 - double(log.time_offset_nsec)*10^-9; 
    rad2.range = sqrt(rad2.x_rel.^2 + rad2.y_rel.^2);
end

% detection number
rad1 = sum_radar_counts(rad1, 4);
if(compare); rad2 = sum_radar_counts(rad2, 4); end

% liveliness
% liv_idx = find_radar_indices(log);
% liveliness_stamp = log.liveliness__state.bag_stamp;


% %% LATENCY FIGURE
% figure('name','Latency')
% tiledlayout(3,1,'Padding','compact');
% 
% axes(f) = nexttile; f=f+1;
% hold on;
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(1)), 'DisplayName','RF');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(3)), 'DisplayName','RR');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(5)), 'DisplayName','RL');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(7)), 'DisplayName','RB');
% grid on; title('radar points rate [s]'); legend; xlim(x_lim);
% 
% axes(f) = nexttile; f=f+1;
% hold on;
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(2)), 'DisplayName','RF');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(4)), 'DisplayName','RR');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(6)), 'DisplayName','RL');
% plot(liveliness_stamp, log.liveliness__state.nodes_states__current_rate(:,liv_idx(8)), 'DisplayName','RB');
% grid on; title('radar udp packages rate [s]'); legend; xlim(x_lim);
% 
% axes(f) = nexttile; f=f+1;
% hold on;
% plot(rad1.stamp,rad1.stamp-rad1.sens_stamp, 'Color', col.radar,'DisplayName',NAME_1);
% grid on; title('radar clustering latency [s]'); legend; xlim(x_lim);


%% STATE FIGURE COG
figure('name','Detections - CoG')
tiledlayout(2,1,'Padding','compact');

% pos x
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp, safe_cols(rad1.x_rel, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare);plot(rad2_sens_stamp, safe_cols(rad2.x_rel, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.x_rel, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('x rel [m]'); xlim(x_lim); ylim([-200 200]);

% pos y
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp, safe_cols(rad1.y_rel, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare); plot(rad2_sens_stamp, safe_cols(rad2.y_rel, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.y_rel, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('y rel [m]'); xlim(x_lim); ylim([-100 100]);

%% STATE FIGURE RANGE
figure('name','Detections - Range')
tiledlayout(2,1,'Padding','compact');

% rho 
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp, safe_cols(rad1.range, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare); plot(rad2_sens_stamp, safe_cols(rad2.range, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.rho, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('range [m]'); xlim(x_lim); ylim([0 200]);

% detections
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp,rad1.count,'Color',col.radar,'DisplayName',NAME_1);
if(compare); plot(rad2_sens_stamp,rad2.count,'Color',col.radar2,'DisplayName',NAME_2); end
grid on; title('Detections [#]'); xlim(x_lim);


%% STATE FIGURE MAP
figure('name','Detections - Map')
tiledlayout(2,1,'Padding','compact');

% pos X
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp(:), safe_cols(rad1.x_map, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare); plot(rad2_sens_stamp, safe_cols(rad2.x_map, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.x_map, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('x map [m]'); xlim(x_lim);

% pos y
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp, safe_cols(rad1.y_map, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare); plot(rad2_sens_stamp, safe_cols(rad2.y_map, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.y_map, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('y map [m]'); xlim(x_lim);

%% STATE FIGURE SPEED
figure('name','Detections - Speed')
tiledlayout(2,1,'Padding','compact');

% rho dot
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp, safe_cols(rad1.rho_dot, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare); plot(rad2_sens_stamp, safe_cols(rad2.rho_dot, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.rho_dot, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('rho dot [m/s]'); xlim(x_lim);

% vx
axes(f) = nexttile([1,1]); f=f+1;
hold on;
plot(rad1.sens_stamp, safe_cols(rad1.vx, rad1.max_det), 'o', 'MarkerFaceColor', col.radar, 'MarkerEdgeColor', col.radar, 'MarkerSize', sz, 'DisplayName', NAME_1);
if(compare); plot(rad2_sens_stamp, safe_cols(rad2.vx, rad2.max_det), 'o', 'MarkerFaceColor', col.radar2, 'MarkerEdgeColor', col.radar2, 'MarkerSize', sz, 'DisplayName', NAME_2); end
if(ground_truth); plot(gt_timestamp, gt.speed, 'Color', col.ref, 'DisplayName', 'gt'); end
grid on; title('vx [m/s]'); xlim(x_lim); ylim([0 inf]);

linkaxes(axes,'x');


%% MAP GUI
fig = figure('Name','MAP');

c = uicontrol('Style','pushbutton');
c.String = {'Refresh'};
c.Callback = @refreshTimeButtonPushed;

function refreshTimeButtonPushed(src,event)
    axes = evalin('base', 'axes');
    traj_db = evalin('base', 'trajDatabase');
    ground_truth = evalin('base', 'ground_truth');
    compare = evalin('base', 'compare');
    col = evalin('base', 'col');
    rad1 = evalin('base', 'rad1');
    bridge_position = evalin('base', 'BRIDGE_POSITION');
    if(ground_truth); gt=evalin('base','gt'); end
    if(ground_truth); gt_timestamp = evalin('base','gt_timestamp'); end
    if(compare); rad2 = evalin('base', 'rad2'); end
    if(compare); rad2_sens_stamp = evalin('base', 'rad2_sens_stamp'); end
    
    t_lim=xlim(axes(1));
    t1_rad_clust = find(rad1.sens_stamp>t_lim(1),1);
    tend_rad_clust = find(rad1.sens_stamp<t_lim(2),1,'last');
    if(compare)
        t1_rad_clust2 = find(rad2_sens_stamp>t_lim(1),1);
        tend_rad_clust2 = find(rad2_sens_stamp<t_lim(2),1,'last');
    end
    if(ground_truth)
        t1_gt_ref = find(gt_timestamp>t_lim(1),1);
        tend_gt_ref = find(gt_timestamp<t_lim(2),1,'last');
    end

    subplot(1,1,1)
    cla reset 
    title('map')
    hold on
    grid on
    xlabel('x[m]')
    ylabel('y[m]')
    axis 'equal'

    % plot track lines
    id_left = length(traj_db) - 2;
    id_right = length(traj_db) - 1;
    plot(traj_db(id_left).x, traj_db(id_left).y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
    plot(traj_db(id_right).x, traj_db(id_right).y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');

    plot(rad1.x_map(t1_rad_clust:tend_rad_clust), rad1.y_map(t1_rad_clust:tend_rad_clust),'.','markersize',20,'Color',col.radar,'displayname','Rad Clust');
    if(compare); plot(rad2.x_map(t1_rad_clust2:tend_rad_clust2), rad2.y_map(t1_rad_clust2:tend_rad_clust2),'.','markersize',20,'Color',col.radar2,'displayname','Rad Clust'); end
    if(ground_truth); plot(gt.x_map(t1_gt_ref:tend_gt_ref),gt.y_map(t1_gt_ref:tend_gt_ref),'Color',col.ref,'DisplayName','Grond Truth'); end

    % Draw draggable bridge regions using THRESHOLD as the radius in meters.
    circle_angle = linspace(0, 2*pi, 64);
    for bridge_idx = 1:numel(bridge_position.X_MAP)
        if bridge_idx == 1
            handle_visibility = 'on';
        else
            handle_visibility = 'off';
        end

        patch( ...
            bridge_position.X_MAP(bridge_idx) + bridge_position.THRESHOLD*cos(circle_angle), ...
            bridge_position.Y_MAP(bridge_idx) + bridge_position.THRESHOLD*sin(circle_angle), ...
            'red', ...
            'FaceAlpha', 0.3, ...
            'EdgeColor', 'black', ...
            'HandleVisibility', handle_visibility, ...
            'DisplayName', 'Bridge positions', ...
            'PickableParts', 'all', ...
            'HitTest', 'on', ...
            'ButtonDownFcn', @(circle, event) startBridgeDrag(circle, event, bridge_idx));
    end

    legend show
end

function startBridgeDrag(circle, ~, bridge_idx)
    fig = ancestor(circle, 'figure');
    if ~strcmp(fig.SelectionType, 'normal')
        return;
    end

    ax = ancestor(circle, 'axes');
    bridge_position = evalin('base', 'BRIDGE_POSITION');
    center = [ ...
        bridge_position.X_MAP(bridge_idx), ...
        bridge_position.Y_MAP(bridge_idx)];
    cursor_position = ax.CurrentPoint(1, 1:2);

    drag_state.circle = circle;
    drag_state.axes = ax;
    drag_state.bridge_idx = bridge_idx;
    drag_state.cursor_offset = center - cursor_position;
    drag_state.circle_x_offset = circle.XData - center(1);
    drag_state.circle_y_offset = circle.YData - center(2);
    drag_state.center = center;

    setappdata(fig, 'BridgeDragState', drag_state);
    fig.Pointer = 'fleur';
    fig.WindowButtonMotionFcn = @moveBridgeCircle;
    fig.WindowButtonUpFcn = @finishBridgeDrag;
end

function moveBridgeCircle(fig, ~)
    if ~isappdata(fig, 'BridgeDragState')
        return;
    end

    drag_state = getappdata(fig, 'BridgeDragState');
    cursor_position = drag_state.axes.CurrentPoint(1, 1:2);
    drag_state.center = cursor_position + drag_state.cursor_offset;

    drag_state.circle.XData = drag_state.center(1) + drag_state.circle_x_offset;
    drag_state.circle.YData = drag_state.center(2) + drag_state.circle_y_offset;
    setappdata(fig, 'BridgeDragState', drag_state);
    drawnow limitrate nocallbacks;
end

function finishBridgeDrag(fig, ~)
    if ~isappdata(fig, 'BridgeDragState')
        return;
    end

    moveBridgeCircle(fig, []);
    drag_state = getappdata(fig, 'BridgeDragState');

    bridge_position = evalin('base', 'BRIDGE_POSITION');
    bridge_position.X_MAP(drag_state.bridge_idx) = drag_state.center(1);
    bridge_position.Y_MAP(drag_state.bridge_idx) = drag_state.center(2);
    assignin('base', 'BRIDGE_POSITION', bridge_position);

    fprintf('\nBridge circle %d moved:\n', drag_state.bridge_idx);
    fprintf('BRIDGE_POSITION.X_MAP(%d) = %.3f;\n', ...
        drag_state.bridge_idx, drag_state.center(1));
    fprintf('BRIDGE_POSITION.Y_MAP(%d) = %.3f;\n', ...
        drag_state.bridge_idx, drag_state.center(2));

    rmappdata(fig, 'BridgeDragState');
    fig.Pointer = 'arrow';
    fig.WindowButtonMotionFcn = '';
    fig.WindowButtonUpFcn = '';
end

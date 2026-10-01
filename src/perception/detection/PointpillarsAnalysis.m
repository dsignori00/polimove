close all
close all
clearvars -except log log2 log_ref trajDatabase

ground_truth = false;
compare      = true;

NAME_1 = "PointPillars";
NAME_2 = "PointPillars - 2";
log_tt = false;
R2D = 57.2958;

%% Paths
addpath("../../common/utilities/")
addpath("../../../common/constants/")
addpath("../../common/plot/")
addpath("../utils/")
addpath("../../../common/graphic_tools/")
normal_path = "../../bags";
opp_path = "../opponent_gps/mat";
% load_colors;
graphics_options
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

% load log2
if (compare && ~exist('log2','var'))
    [file,path] = uigetfile(fullfile(normal_path,'*.mat'),'Load log 2');
    tmp = load(fullfile(path,file));
    log2 = tmp.log;
end

if(ground_truth)
    if (~exist('gt','var'))
        [file,path] = uigetfile(fullfile(opp_path,'*.mat'),'Load ground truth log');
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

%% NAMING

col.tt = colors.blue{2};
% col.pointpillars = colors.green{3};
col.pointpillars = [7/255, 157/255, 104/255];
col.pointpillars2 = colors.purple{1};
col.ref = '#000000';
sz=3; % Marker size
f=1;
x_lim = [0 inf];

% POINTPILLARS DETECTIONS
lid_pp = get_pointpillars_fields(log);
t0 = log.time_offset_nsec;
t02 = inf;
if(compare) lid_pp2 = get_pointpillars_fields(log2); end
if(compare) t02 = log2.time_offset_nsec; end

% TARGET TRACKING
if (log_tt)
    tt = load_tt(log);
    max_opp = tt.max_opp;
    
    if(compare) 
        tt2 = load_tt(log2); 
        max_opp2 = tt2.max_opp;
        max_opp = max(max_opp, max_opp2);
    end

else
    max_opp = 15;
end

%% PROCESSING 

% Range Computation
lid_pp.range = sqrt(lid_pp.x_rel.^2 + lid_pp.y_rel.^2);
if (compare) lid_pp2.range = sqrt(lid_pp2.x_rel.^2 + lid_pp2.y_rel.^2); end
if(log_tt)
    tt.range = sqrt(tt.x_rel.^2 + tt.y_rel.^2);
    if (compare) tt2.range =  sqrt(tt2.x_rel.^2 + tt2.y_rel.^2); end
end

if(ground_truth)
    gt.range = sqrt(gt.x_rel.^2 + gt.y_rel.^2);
end


% compute database heading
if (~exist('lid_pp.idx','var'))
    for i = 1:lid_pp.max_det
        opp_pos = [lid_pp.x_map(:,i), lid_pp.y_map(:,i)];
        [opp_yaw_db, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
        lid_pp.yaw_map_db(:,i) = opp_yaw_db;
        lid_pp.idx(:,i) = opp_idx;
    end
end
if(compare)
    if (~exist('lid_pp2.idx','var'))
        for i = 1:lid_pp2.max_det
            opp_pos = [lid_pp2.x_map(:,i), lid_pp2.y_map(:,i)];
            [opp_yaw_db, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
            lid_pp2.yaw_map_db(:,i) = opp_yaw_db;
            lid_pp2.idx(:,i) = opp_idx;
        end
    end
end

% align timestamp
[t0_min, idx_min] = min([t0,t02]);
lid_pp.sens_stamp = lid_pp.sens_stamp + double(t0)*1e-9 - double(t0_min)*1e-9;
lid_pp.stamp = lid_pp.stamp + double(t0)*1e-9 - double(t0_min)*1e-9;
if (compare) lid_pp2.sens_stamp = lid_pp2.sens_stamp + double(t02)*1e-9 - double(t0_min)*1e-9; end
if (compare) lid_pp2.stamp = lid_pp2.stamp + double(t02)*1e-9 - double(t0_min)*1e-9; end

if(ground_truth); gt_timestamp = (gt.timestamp - double(t0_min))*10^-9; end
    



%% LATENCY FIGURE
figure('name','Latency')
tiledlayout(1,1,'Padding','compact');

axes(f) = nexttile; f=f+1; hold on;
plot(lid_pp.stamp,lid_pp.stamp - lid_pp.sens_stamp);
if (compare) plot(lid_pp2.stamp,lid_pp2.stamp - lid_pp2.sens_stamp); end
grid on; title('lidar_pp [s]')

%% STATE FIGURE MAP
figure('name', 'Detections - Map');
tiledlayout(2,1,'Padding','compact');

% pos x
axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(lid_pp.sens_stamp,lid_pp.x_map(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1);
if (compare) plot(lid_pp2.sens_stamp,lid_pp2.x_map(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp,tt.x_map(:,1:max_opp),'Color',col.tt,'DisplayName','tt'); end
if(ground_truth ); plot(gt.stamp,gt.x_map,'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('x map [m]'); 

% pos y
axes(f) = nexttile([1,1]); f=f+1; hold on; 
plot(lid_pp.sens_stamp,lid_pp.y_map(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1);
if(compare) plot(lid_pp2.sens_stamp,lid_pp2.y_map(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp,tt.y_map(:,1:max_opp),'Color',col.tt,'DisplayName','tt'); end
if(ground_truth); plot(gt.stamp,gt.y_map,'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('y map [m]'); 

%% YAW
figure('name', 'Detection - Yaw');
tiledlayout(2,1,'Padding','compact')

% if(max(max(lid_pp.valid_yaw)))
%     type = "MEAS";
%     if(use_hd_report)
%         type2 = "DB";
%         yaw_map = wrapTo2Pi(lid_pp.hd_report.traj_global_heading);
%         yaw_rel = wrapToPi(lid_pp.hd_report.traj_relative_heading);
% 
%         if(compare)
%             yaw_map2 = wrapTo2Pi(lid_pp2.hd_report.traj_global_heading);
%             yaw_rel2 = wrapToPi(lid_pp2.hd_report.traj_relative_heading);
%         end
%     end
% else
%     type = "DB";
%     type2 = "MEAS";
%     if(use_hd_report)
%         yaw_map = wrapTo2Pi(lid_pp.hd_report.measured_global_heading);
%         yaw_rel = wrapToPi(lid_pp.hd_report.measured_relative_heading);
%         if(compare)
%             yaw_map2 = wrapTo2Pi(lid_pp2.hd_report.measured_global_heading);
%             yaw_rel2 = wrapToPi(lid_pp2.hd_report.measured_relative_heading);
%         end
%     end
% end



%===== Absolute Yaw ===%
lid_pp.yaw_map = wrapTo2Pi(lid_pp.yaw_map);
if(compare) lid_pp2.yaw_map = wrapTo2Pi(lid_pp2.yaw_map); end
axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(lid_pp.sens_stamp, wrapToPi(lid_pp.yaw_map_db)* R2D,'o','MarkerFaceColor',colors.red{2},'MarkerEdgeColor',colors.red{2},'MarkerSize',sz,'DisplayName','DB');
plot(lid_pp.sens_stamp, lid_pp.yaw_map(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1);
if(compare) plot(lid_pp2.sens_stamp,lid_pp2.yaw_map(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.pointpillars2,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp,wrapTo2Pi(tt.yaw_map(:,1:max_opp))* R2D,'Color',col.tt,'DisplayName','tt','LineWidth',2.0); end
if(ground_truth)
    gt.yaw_map = wrapTo2Pi(gt.yaw_map);
    plot(gt.stamp,gt.yaw_map* R2D,'Color',col.ref,'DisplayName','Ground Truth');
end
grid on; title('yaw map [deg]'); legend show


%===== Relative Yaw ===%

% lid_pp.yaw_rel = mod(lid_pp.yaw_rel,2*pi);

axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(lid_pp.sens_stamp,lid_pp.yaw_rel(:,1:max_opp)* R2D,'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1);
if(compare) plot(lid_pp2.sens_stamp,lid_pp2.yaw_rel(:,1:max_opp)* R2D,'o','MarkerFaceColor',col.pointpillars2,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp,wrapToPi(tt.yaw_rel(:,1:max_opp))* R2D,'Color',col.tt,'DisplayName','tt', 'LineWidth',2.0); end
if(ground_truth)
    gt.yaw_rel = wrapToPi(gt.yaw_rel);
    plot(gt.stamp,gt.yaw_rel* R2D,'Color',col.ref,'DisplayName','Ground Truth');
end
grid on; title('yaw relative [deg]');



%% STATE FIGURE REL
figure('name', 'PP - CoG');
tiledlayout(2,1,'Padding','compact');

% pos x
axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(lid_pp.sens_stamp,lid_pp.x_rel(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1);
if (compare) plot(lid_pp2.sens_stamp,lid_pp2.x_rel(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars2,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp, tt.x_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth); plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('x rel [m]'); 

% pos y
axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(lid_pp.sens_stamp,lid_pp.y_rel(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1);
if(compare) plot(lid_pp2.sens_stamp,lid_pp2.y_rel(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars2,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp, tt.y_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth); plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('y rel [m]'); 

%% RANGE
figure('name', 'Filter - Range');
tiledlayout(2,1,'Padding','compact');

% rho 
axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(lid_pp.sens_stamp,lid_pp.range(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars,'MarkerEdgeColor',col.pointpillars,'MarkerSize',sz,'DisplayName',NAME_1); 
if(compare) plot(lid_pp2.sens_stamp,lid_pp2.range(:,1:max_opp),'o','MarkerFaceColor',col.pointpillars2,'MarkerEdgeColor',col.pointpillars2,'MarkerSize',sz,'DisplayName',NAME_2); end
if(log_tt) plot(tt.stamp, tt.range(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth); plot(gt.stamp, gt.rho, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('range [m]'); 



% count
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(log_tt)
    area(tt.stamp,tt.buffer.lid_pp(:,1:max_opp),'FaceColor',col.pointpillars,'EdgeColor',col.pointpillars,'DisplayName','Pointpillars');
else
    area(lid_pp.sens_stamp, lid_pp.count, 'FaceColor',col.pointpillars, 'EdgeColor',col.pointpillars, 'DisplayName','Pointpillars');
    if(compare) area(lid_pp2.sens_stamp, lid_pp2.count, 'FaceColor',col.pointpillars2, 'EdgeColor',col.pointpillars2, 'DisplayName','Pointpillars'); end
end
grid on; title('Count'); 

%% CONFIDENCE
figure('name', 'PP - Condidence');
tiledlayout(2,3,'Padding','compact');

scores = lid_pp.score(:,1:max_opp)'; scores = scores(:);
x_rel = lid_pp.x_rel(:,1:max_opp)'; x_rel= x_rel(:);
y_rel = lid_pp.y_rel(:,1:max_opp)'; y_rel= y_rel(:);
t = repelem(lid_pp.sens_stamp, max_opp);
colormap turbo

% pos x
axes(f) = nexttile([1,2]); f=f+1; hold on;
scatter(t,x_rel, sz*7,scores,'filled');
if(ground_truth); plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('x rel [m]'); 
colorbar

% pos y
axes(f) = nexttile([1,2]); f=f+1; hold on;
scatter(t,y_rel, sz*7,scores,'filled');
if(log_tt) plot(tt.stamp, tt.y_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth); plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('y rel [m]'); 
colorbar

% range - confidence
nexttile([2,1]); hold on;
scatter(lid_pp.range(:,1:max_opp), lid_pp.score(:,1:max_opp))
xlabel("range [m]")
ylabel("score [-]")

if(compare)
    figure('name', 'PP - Condidence 2');
    tiledlayout(2,3,'Padding','compact');
    
    scores2 = lid_pp2.score(:,1:max_opp)'; scores2 = scores2(:);
    x_rel2 = lid_pp2.x_rel(:,1:max_opp)'; x_rel2= x_rel2(:);
    y_rel2 = lid_pp2.y_rel(:,1:max_opp)'; y_rel2= y_rel2(:);
    t2 = repelem(lid_pp2.sens_stamp, max_opp);
    colormap turbo
    
    % pos x
    axes(f) = nexttile([1,2]); f=f+1; hold on;
    scatter(t2,x_rel2, sz*7,scores2,'filled');
    if(ground_truth); plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
    grid on; title('x rel [m]'); 
    colorbar
    
    % pos y
    axes(f) = nexttile([1,2]); f=f+1; hold on;
    scatter(t2,y_rel2, sz*7,scores2,'filled');
    if(log_tt) plot(tt2.stamp, tt2.y_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
    if(ground_truth); plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
    grid on; title('y rel [m]'); 
    colorbar
    
    % range - confidence
    nexttile([2,1]); hold on;
    scatter(lid_pp2.range(:,1:max_opp), lid_pp2.score(:,1:max_opp))
    xlabel("range [m]")
    ylabel("score [-]")

end

%% MAP
linkaxes(axes,'x');

fig = figure('name','MAP');

c = uicontrol('Style','pushbutton');
c.String = {'Refresh'};
c.Callback = @refreshTimeButtonPushed;


function refreshTimeButtonPushed(src,event)
    axes = evalin('base', 'axes');
    traj_db = evalin('base', 'trajDatabase');
    ground_truth = evalin('base', 'ground_truth');
    col = evalin('base', 'col');
    log_tt = evalin('base','log_tt');
    compare = evalin('base','compare');
    sz = evalin('base','sz');
    lid_pp = evalin('base', 'lid_pp');
    if(log_tt) tt=evalin('base','tt'); end
    if(ground_truth); gt =evalin('base','gt'); end
    if(compare); lid_pp2 =evalin('base','lid_pp2'); end

    t_lim=xlim(axes(1));
    t1_lid_pp = find(lid_pp.sens_stamp>t_lim(1),1);
    tend_lid_pp = find(lid_pp.sens_stamp<t_lim(2),1,'last');
    if(log_tt) t1_tt = find(tt.stamp>t_lim(1),1); end
    if(log_tt) tend_tt = find(tt.stamp<t_lim(2),1,'last'); end
    if(ground_truth)
        t1_tt_ref = find(gt.stamp>t_lim(1),1);
        tend_tt_ref = find(gt.stamp<t_lim(2),1,'last');
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
    plot(traj_db(id_left).X, traj_db(id_left).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
    plot(traj_db(id_right).X, traj_db(id_right).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');

    % plot(lid_pp.x_map(t1_lid_pp:tend_lid_pp), lid_pp.y_map(t1_lid_pp:tend_lid_pp),'.','markersize',20,'Color',col.pointpillars,'displayname','Lid PP');
    scatter(lid_pp.x_map(t1_lid_pp:tend_lid_pp), lid_pp.y_map(t1_lid_pp:tend_lid_pp),20,'filled','DisplayName','Lidar PP','MarkerEdgeColor',col.pointpillars,'MarkerFaceColor',col.pointpillars);
    if(compare) scatter(lid_pp2.x_map(t1_lid_pp:tend_lid_pp), lid_pp2.y_map(t1_lid_pp:tend_lid_pp),20,'filled','DisplayName','Lidar PP2','MarkerEdgeColor',col.pointpillars2,'MarkerFaceColor',col.pointpillars2); end

    if(log_tt) plot(tt.x_map(t1_tt:tend_tt),tt.y_map(t1_tt:tend_tt),'Color',col.tt,'DisplayName','tt'); end
    if(ground_truth)
        plot(gt.x_map(t1_tt_ref:tend_tt_ref),gt.y_map(t1_tt_ref:tend_tt_ref),'Color',col.ref,'DisplayName','Grond Truth');
    end
    legend show
end

 %% CONFIDENCE MAP
% linkaxes(axes,'x');
% 
% fig = figure('name','CONFIDENCE MAP');
% 
% c = uicontrol('Style','pushbutton');
% c.String = {'Refresh'};
% c.Callback = @refreshTimeButtonPushed;
% 
% 
% function refreshTimeButtonPushed(src,event)
%     axes = evalin('base', 'axes');
%     traj_db = evalin('base', 'trajDatabase');
%     ground_truth = evalin('base', 'ground_truth');
%     col = evalin('base', 'col');
%     log_tt = evalin('base','log_tt');
%     sz = evalin('base','sz');
%     lid_pp = evalin('base', 'lid_pp');
%     if(log_tt) tt=evalin('base','tt'); end
%     if(ground_truth); gt =evalin('base','gt'); end
% 
%     t_lim=xlim(axes(1));
%     t1_lid_pp = find(lid_pp.sens_stamp>t_lim(1),1);
%     tend_lid_pp = find(lid_pp.sens_stamp<t_lim(2),1,'last');
%     if(log_tt) t1_tt = find(tt.stamp>t_lim(1),1); end
%     if(log_tt) tend_tt = find(tt.stamp<t_lim(2),1,'last'); end
%     if(ground_truth)
%         t1_tt_ref = find(gt.stamp>t_lim(1),1);
%         tend_tt_ref = find(gt.stamp<t_lim(2),1,'last');
%     end
% 
%     subplot(1,1,1)
%     cla reset 
%     title('map')
%     hold on
%     grid on
%     xlabel('x[m]')
%     ylabel('y[m]')
%     axis 'equal'
% 
%     % plot track lines
%     id_left = length(traj_db) - 2;
%     id_right = length(traj_db) - 1;
%     plot(traj_db(id_left).X, traj_db(id_left).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
%     plot(traj_db(id_right).X, traj_db(id_right).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
% 
%     % plot(lid_pp.x_map(t1_lid_pp:tend_lid_pp), lid_pp.y_map(t1_lid_pp:tend_lid_pp),'.','markersize',20,'Color',col.pointpillars,'displayname','Lid PP');
%     scatter(lid_pp.x_map(t1_lid_pp:tend_lid_pp), lid_pp.y_map(t1_lid_pp:tend_lid_pp),20, lid_pp.score(t1_lid_pp:tend_lid_pp),'filled','DisplayName','Lidar PP');
%     if(log_tt) plot(tt.x_map(t1_tt:tend_tt),tt.y_map(t1_tt:tend_tt),'Color',col.tt,'DisplayName','tt'); end
%     if(ground_truth)
%         plot(gt.x_map(t1_tt_ref:tend_tt_ref),gt.y_map(t1_tt_ref:tend_tt_ref),'Color',col.ref,'DisplayName','Grond Truth');
%     end
%     legend show
%     colormap turbo
%     colorbar
% end

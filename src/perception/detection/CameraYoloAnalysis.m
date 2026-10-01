close all
close all
clearvars -except log log2 gt trajDatabase

ground_truth = true;
compare      = true;

CAMONLY = false;
CAMENH  = true;

NAME_1 = "CameraYolo";
NAME_2 = "CameraYolo - 2";
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
% col.camyolo = colors.green{3};
col.camyolo = colors.yellow{1};
col.camyolo2 = colors.yellow{2};
col.camenh = colors.orange{1};
col.camenh2 = colors.orange{2};
col.ref = '#000000';
sz=3; % Marker size
f=1;
x_lim = [0 inf];

% CAMERAYOLO DETECTIONS
if(CAMONLY) cam_yolo = get_camerayolo_fields(log,3); end
if(CAMENH) cam_yolo_enh = get_camerayolo_fields(log,4); end
t0 = log.time_offset_nsec;
t02 = inf;
if(compare && CAMONLY) cam_yolo2 = get_camerayolo_fields(log2,3); end
if(compare && CAMENH) cam_yolo_enh2 = get_camerayolo_fields(log2,4); end
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
if (CAMONLY) cam_yolo.range = sqrt(cam_yolo.x_rel.^2 + cam_yolo.y_rel.^2); end
if (CAMENH)  cam_yolo_enh.range = sqrt(cam_yolo_enh.x_rel.^2 + cam_yolo_enh.y_rel.^2); end
if (compare && CAMONLY) cam_yolo2.range = sqrt(cam_yolo2.x_rel.^2 + cam_yolo2.y_rel.^2); end
if (compare && CAMENH) cam_yolo_enh2.range = sqrt(cam_yolo_enh2.x_rel.^2 + cam_yolo_enh2.y_rel.^2); end
if(log_tt)
    tt.range = sqrt(tt.x_rel.^2 + tt.y_rel.^2);
    if (compare) tt2.range =  sqrt(tt2.x_rel.^2 + tt2.y_rel.^2); end
end

if(ground_truth)
    gt.range = sqrt(gt.x_rel.^2 + gt.y_rel.^2);
end


% compute database heading
if (~exist('cam_yolo.idx','var') && CAMONLY)
    for i = 1:cam_yolo.max_det
        opp_pos = [cam_yolo.x_map(:,i), cam_yolo.y_map(:,i)];
        [opp_yaw_db, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
        cam_yolo.yaw_map_db(:,i) = opp_yaw_db;
        cam_yolo.idx(:,i) = opp_idx;
    end
end
if (~exist('cam_yolo_enh.idx','var') && CAMENH)
    for i = 1:cam_yolo_enh.max_det
        opp_pos = [cam_yolo_enh.x_map(:,i), cam_yolo_enh.y_map(:,i)];
        [opp_yaw_db, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
        cam_yolo_enh.yaw_map_db(:,i) = opp_yaw_db;
        cam_yolo_enh.idx(:,i) = opp_idx;
    end
end
if(compare)
    if (~exist('cam_yolo2.idx','var') && CAMONLY)
        for i = 1:cam_yolo2.max_det
            opp_pos = [cam_yolo2.x_map(:,i), cam_yolo2.y_map(:,i)];
            [opp_yaw_db, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
            cam_yolo2.yaw_map_db(:,i) = opp_yaw_db;
            cam_yolo2.idx(:,i) = opp_idx;
        end
    end
    if (~exist('cam_yolo_enh2.idx','var') && CAMENH)
        for i = 1:cam_yolo_enh2.max_det
            opp_pos = [cam_yolo_enh2.x_map(:,i), cam_yolo_enh2.y_map(:,i)];
            [opp_yaw_db, opp_idx] = get_heading(opp_pos, trajDatabase, -1);
            cam_yolo_enh2.yaw_map_db(:,i) = opp_yaw_db;
            cam_yolo_enh2.idx(:,i) = opp_idx;
        end
    end
end

% align timestamp
[t0_min, idx_min] = min([t0,t02]);
if(CAMONLY)
    cam_yolo.sens_stamp = cam_yolo.sens_stamp + double(t0)*1e-9 - double(t0_min)*1e-9; 
    cam_yolo.stamp = cam_yolo.stamp + double(t0)*1e-9 - double(t0_min)*1e-9; 
end
if(CAMENH) 
    cam_yolo_enh.sens_stamp = cam_yolo_enh.sens_stamp + double(t0)*1e-9 - double(t0_min)*1e-9;
    cam_yolo_enh.stamp = cam_yolo_enh.stamp + double(t0)*1e-9 - double(t0_min)*1e-9;
end 
if (compare) 
    if(CAMONLY)
        cam_yolo2.sens_stamp = cam_yolo2.sens_stamp + double(t02)*1e-9 - double(t0_min)*1e-9;
        cam_yolo2.stamp = cam_yolo2.stamp + double(t02)*1e-9 - double(t0_min)*1e-9; 
    end
     if(CAMENH)
        cam_yolo_enh2.sens_stamp = cam_yolo_enh2.sens_stamp + double(t02)*1e-9 - double(t0_min)*1e-9;
        cam_yolo_enh2.stamp = cam_yolo_enh2.stamp + double(t02)*1e-9 - double(t0_min)*1e-9; 
    end
end

if(ground_truth); gt_timestamp = (gt.timestamp - double(t0_min))*10^-9; end
    



%% LATENCY FIGURE
figure('name','Latency')
tiledlayout(2,1,'Padding','compact');

axes(f) = nexttile; f=f+1; hold on;
if(CAMONLY) plot(cam_yolo.stamp, cam_yolo.stamp - cam_yolo.sens_stamp,'Color',col.camyolo,'DisplayName',NAME_2); end
if (compare && CAMONLY) plot(cam_yolo2.stamp, cam_yolo2.stamp - cam_yolo2.sens_stamp,'Color', col.camyolo2,'DisplayName',NAME_1); end
grid on; title('cam_yolo [s]')
legend show
axes(f) = nexttile; f=f+1; hold on;
if(CAMENH) plot(cam_yolo_enh.stamp,cam_yolo_enh.stamp - cam_yolo_enh.sens_stamp,'Color',col.camenh,'DisplayName',NAME_2); end
if (compare && CAMENH) plot(cam_yolo_enh2.stamp,cam_yolo_enh2.stamp - cam_yolo_enh2.sens_stamp,'Color', col.camenh2,'DisplayName',NAME_1); end
grid on; title('cam_yolo [s]')
legend show

%% STATE FIGURE MAP
figure('name', 'Detections - Map');
tiledlayout(2,1,'Padding','compact');

% pos x
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(CAMONLY) 
    h1 = plot(cam_yolo.sens_stamp, cam_yolo.x_map(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if(CAMENH) 
    h1 = plot(cam_yolo_enh.sens_stamp, cam_yolo_enh.x_map(:,1:max_opp),'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if (compare) 
    if(CAMONLY)
        h2 = plot(cam_yolo2.sens_stamp,cam_yolo2.x_map(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH)
        h2 = plot(cam_yolo_enh2.sens_stamp,cam_yolo_enh2.x_map(:,1:max_opp),'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
end
if(log_tt) plot(tt.stamp,tt.x_map(:,1:max_opp),'Color',col.tt,'DisplayName','tt'); end
if(ground_truth ); plot(gt_timestamp, gt.x_map,'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('x map [m]'); 
legend show

% pos y
axes(f) = nexttile([1,1]); f=f+1; hold on; 
if(CAMONLY) 
    h1 = plot(cam_yolo.sens_stamp,cam_yolo.y_map(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if(CAMENH) 
    h1 = plot(cam_yolo_enh.sens_stamp,cam_yolo_enh.y_map(:,1:max_opp),'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if (compare) 
    if(CAMONLY)
        h2 = plot(cam_yolo2.sens_stamp,cam_yolo2.y_map(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH)
        h2 = plot(cam_yolo_enh2.sens_stamp,cam_yolo_enh2.y_map(:,1:max_opp),'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
end
if(log_tt) plot(tt.stamp,tt.y_map(:,1:max_opp),'Color',col.tt,'DisplayName','tt'); end
if(ground_truth) plot(gt_timestamp,gt.y_map,'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('y map [m]'); 
legend show
%% YAW
figure('name', 'Detection - Yaw');
tiledlayout(2,1,'Padding','compact')

%===== Absolute Yaw ===%
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(CAMONLY)
    cam_yolo.yaw_map = wrapTo2Pi(cam_yolo.yaw_map);
    h2 = plot(cam_yolo.sens_stamp, cam_yolo.yaw_map(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h2(2:end), 'HandleVisibility', 'off');
end
if(CAMENH)
    cam_yolo_enh.yaw_map = wrapTo2Pi(cam_yolo_enh.yaw_map);
    h2 = plot(cam_yolo_enh.sens_stamp, cam_yolo_enh.yaw_map(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h2(2:end), 'HandleVisibility', 'off');
end
if(compare) 
    if(CAMONLY)
        cam_yolo2.yaw_map = wrapTo2Pi(cam_yolo2.yaw_map);
        h2 = plot(cam_yolo2.sens_stamp, cam_yolo2.yaw_map(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_2);
        set(h2(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH)
        cam_yolo_enh2.yaw_map = wrapTo2Pi(cam_yolo_enh2.yaw_map);
        h2 = plot(cam_yolo_enh2.sens_stamp, cam_yolo_enh2.yaw_map(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2);
        set(h2(2:end), 'HandleVisibility', 'off');
    end
end
if(ground_truth)
    gt.yaw_map = wrapTo2Pi(gt.yaw_map);
    plot(gt_timestamp,gt.yaw_map* R2D,'Color',col.ref,'DisplayName','Ground Truth');
end
grid on; title('yaw map [deg]'); legend show


%===== Relative Yaw ===%

axes(f) = nexttile([1,1]); f=f+1; hold on;
if(CAMONLY)
    cam_yolo.yaw_rel = wrapToPi(cam_yolo.yaw_rel);
    h2 = plot(cam_yolo.sens_stamp, cam_yolo.yaw_rel(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h2(2:end), 'HandleVisibility', 'off');
end
if(CAMENH)
    cam_yolo_enh.yaw_rel = wrapToPi(cam_yolo_enh.yaw_rel);
    h2 = plot(cam_yolo_enh.sens_stamp, cam_yolo_enh.yaw_rel(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h2(2:end), 'HandleVisibility', 'off');
end
if(compare) 
    if(CAMONLY)
        cam_yolo2.yaw_rel = wrapToPi(cam_yolo2.yaw_rel);
        h2 = plot(cam_yolo2.sens_stamp, cam_yolo2.yaw_rel(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camyolo2,'MarkerEdgeColor',col.camyolo2,'MarkerSize',sz,'DisplayName',NAME_2);
        set(h2(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH)
        cam_yolo_enh2.yaw_rel = wrapToPi(cam_yolo_enh2.yaw_rel);
        h2 = plot(cam_yolo_enh2.sens_stamp, cam_yolo_enh2.yaw_rel(:,1:max_opp) * R2D,'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2);
        set(h2(2:end), 'HandleVisibility', 'off');
    end
end
if(log_tt) plot(tt.stamp,wrapToPi(tt.yaw_rel(:,1:max_opp)) * R2D,'Color',col.tt,'DisplayName','tt', 'LineWidth',2.0); end
if(ground_truth)
    gt.yaw_rel = wrapToPi(gt.yaw_rel);
    plot(gt_timestamp,gt.yaw_rel * R2D,'Color',col.ref,'DisplayName','Ground Truth');
end
grid on; title('yaw relative [deg]'); legend show

%% STATE FIGURE REL
figure('name', 'Detection - CoG');
tiledlayout(2,1,'Padding','compact');

% pos x
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(CAMONLY) 
    h1 = plot(cam_yolo.sens_stamp, cam_yolo.x_rel(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if(CAMENH) 
    h1 = plot(cam_yolo_enh.sens_stamp, cam_yolo_enh.x_rel(:,1:max_opp),'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if (compare) 
    if(CAMONLY)
        h2 = plot(cam_yolo2.sens_stamp,cam_yolo2.x_rel(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH)
        h2 = plot(cam_yolo_enh2.sens_stamp,cam_yolo_enh2.x_rel(:,1:max_opp),'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
end
if(log_tt) plot(tt.stamp, tt.x_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth) plot(gt_timestamp, gt.x_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('x rel [m]'); 
legend show

% pos y
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(CAMONLY) 
    h1 = plot(cam_yolo.sens_stamp, cam_yolo.y_rel(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if(CAMENH) 
    h1 = plot(cam_yolo_enh.sens_stamp, cam_yolo_enh.y_rel(:,1:max_opp),'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off')
end
if (compare) 
    if(CAMONLY)
        h2 = plot(cam_yolo2.sens_stamp,cam_yolo2.y_rel(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH)
        h2 = plot(cam_yolo_enh2.sens_stamp,cam_yolo_enh2.y_rel(:,1:max_opp),'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2); 
        set(h2(2:end), 'HandleVisibility', 'off');
    end
end
if(log_tt) plot(tt.stamp, tt.y_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth) plot(gt_timestamp, gt.y_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('y rel [m]'); 
legend show

%% RANGE
figure('name', 'Filter - Range');
tiledlayout(2,1,'Padding','compact');

% rho 
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(CAMONLY) 
    h1 = plot(cam_yolo.sens_stamp,cam_yolo.range(:,1:max_opp),'o','MarkerFaceColor',col.camyolo,'MarkerEdgeColor',col.camyolo,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off');
end
if(CAMENH) 
    h1 = plot(cam_yolo_enh.sens_stamp,cam_yolo_enh.range(:,1:max_opp),'o','MarkerFaceColor',col.camenh,'MarkerEdgeColor',col.camenh,'MarkerSize',sz,'DisplayName',NAME_1);
    set(h1(2:end), 'HandleVisibility', 'off');
end
if(compare)
    if(CAMONLY) 
        h1 = plot(cam_yolo2.sens_stamp,cam_yolo2.range(:,1:max_opp),'o','MarkerFaceColor',col.camyolo2,'MarkerEdgeColor',col.camyolo2,'MarkerSize',sz,'DisplayName',NAME_2);
        set(h1(2:end), 'HandleVisibility', 'off');
    end
    if(CAMENH) 
        h1 = plot(cam_yolo_enh2.sens_stamp,cam_yolo_enh2.range(:,1:max_opp),'o','MarkerFaceColor',col.camenh2,'MarkerEdgeColor',col.camenh2,'MarkerSize',sz,'DisplayName',NAME_2);
        set(h1(2:end), 'HandleVisibility', 'off');
    end
end
if(log_tt) plot(tt.stamp, tt.range(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
if(ground_truth); plot(gt_timestamp, gt.rho, 'Color',col.ref,'DisplayName','Ground Truth'); end
grid on; title('range [m]'); legend show



% count
axes(f) = nexttile([1,1]); f=f+1; hold on;
if(log_tt)
    area(tt.stamp,tt.buffer.cam_yolo(:,1:max_opp),'FaceColor',col.camera,'EdgeColor',col.camera,'DisplayName','Camera');
else
    if(CAMONLY) area(cam_yolo.sens_stamp, cam_yolo.count, 'FaceColor',col.camyolo, 'EdgeColor',col.camyolo, 'DisplayName',NAME_1);end
    if(CAMENH) area(cam_yolo_enh.sens_stamp, cam_yolo_enh.count, 'FaceColor',col.camenh, 'EdgeColor',col.camenh, 'DisplayName',NAME_1); end
    
    if(compare)
        if(CAMONLY) area(cam_yolo2.sens_stamp, cam_yolo.count, 'FaceColor',col.camyolo2, 'EdgeColor',col.camyolo2, 'DisplayName',NAME_2); end
        if(CAMENH) area(cam_yolo_enh2.sens_stamp, cam_yolo_enh2.count, 'FaceColor',col.camenh2, 'EdgeColor',col.camenh2, 'DisplayName',NAME_2); end
    end
end
grid on; title('Count'); 
legend show

% %% CONFIDENCE
% figure('name', 'PP - Condidence');
% tiledlayout(2,3,'Padding','compact');

% scores = cam_yolo.score(:,1:max_opp)'; scores = scores(:);
% x_rel = cam_yolo.x_rel(:,1:max_opp)'; x_rel= x_rel(:);
% y_rel = cam_yolo.y_rel(:,1:max_opp)'; y_rel= y_rel(:);
% t = repelem(cam_yolo.sens_stamp, max_opp);
% colormap turbo

% % pos x
% axes(f) = nexttile([1,2]); f=f+1; hold on;
% scatter(t,x_rel, sz*7,scores,'filled');
% if(ground_truth); plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
% grid on; title('x rel [m]'); 
% colorbar

% % pos y
% axes(f) = nexttile([1,2]); f=f+1; hold on;
% scatter(t,y_rel, sz*7,scores,'filled');
% if(log_tt) plot(tt.stamp, tt.y_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
% if(ground_truth); plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
% grid on; title('y rel [m]'); 
% colorbar

% % range - confidence
% nexttile([2,1]); hold on;
% scatter(cam_yolo.range(:,1:max_opp), cam_yolo.score(:,1:max_opp))
% xlabel("range [m]")
% ylabel("score [-]")

% if(compare)
%     figure('name', 'PP - Condidence 2');
%     tiledlayout(2,3,'Padding','compact');
    
%     scores2 = cam_yolo2.score(:,1:max_opp)'; scores2 = scores2(:);
%     x_rel2 = cam_yolo2.x_rel(:,1:max_opp)'; x_rel2= x_rel2(:);
%     y_rel2 = cam_yolo2.y_rel(:,1:max_opp)'; y_rel2= y_rel2(:);
%     t2 = repelem(cam_yolo2.sens_stamp, max_opp);
%     colormap turbo
    
%     % pos x
%     axes(f) = nexttile([1,2]); f=f+1; hold on;
%     scatter(t2,x_rel2, sz*7,scores2,'filled');
%     if(ground_truth); plot(gt.stamp, gt.x_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
%     grid on; title('x rel [m]'); 
%     colorbar
    
%     % pos y
%     axes(f) = nexttile([1,2]); f=f+1; hold on;
%     scatter(t2,y_rel2, sz*7,scores2,'filled');
%     if(log_tt) plot(tt2.stamp, tt2.y_rel(:,1:max_opp), 'Color',col.tt,'DisplayName','tt'); end
%     if(ground_truth); plot(gt.stamp, gt.y_rel, 'Color',col.ref,'DisplayName','Ground Truth'); end
%     grid on; title('y rel [m]'); 
%     colorbar
    
%     % range - confidence
%     nexttile([2,1]); hold on;
%     scatter(cam_yolo2.range(:,1:max_opp), cam_yolo2.score(:,1:max_opp))
%     xlabel("range [m]")
%     ylabel("score [-]")

% end

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
    NAME_1 = evalin('base','NAME_1');
    NAME_2 = evalin('base','NAME_2');
    CAMONLY = evalin('base','CAMONLY');
    CAMENH = evalin('base','CAMENH');
    if(CAMONLY) cam_yolo = evalin('base', 'cam_yolo'); end
    if(CAMENH)  cam_yolo_enh = evalin('base', 'cam_yolo_enh'); end
    cam_yolo_enh = evalin('base', 'cam_yolo_enh');
    cam_yolo_enh2 = evalin('base', 'cam_yolo_enh2');
    if(log_tt) tt=evalin('base','tt'); end
    if(ground_truth); 
        gt =evalin('base','gt'); 
        gt_timestamp = evalin('base','gt_timestamp'); 
    end
    if(compare)
        if(CAMONLY)  cam_yolo2 =evalin('base','cam_yolo2'); end
        if(CAMENH) cam_yolo_enh2 = evalin('base', 'cam_yolo_enh2'); end
    end
    t_lim=xlim(axes(1));
    if(CAMONLY)
        t1_cam_yolo = find(cam_yolo.sens_stamp>t_lim(1),1);
        tend_cam_yolo = find(cam_yolo.sens_stamp<t_lim(2),1,'last');
    end
    if(CAMENH)
        t1_cam_yolo = find(cam_yolo_enh.sens_stamp>t_lim(1),1);
        tend_cam_yolo = find(cam_yolo_enh.sens_stamp<t_lim(2),1,'last');
    end
    if(log_tt) t1_tt = find(tt.stamp>t_lim(1),1); end
    if(log_tt) tend_tt = find(tt.stamp<t_lim(2),1,'last'); end
    if(ground_truth)
        t1_tt_ref = find(gt_timestamp>t_lim(1),1);
        tend_tt_ref = find(gt_timestamp<t_lim(2),1,'last');
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

    % plot(cam_yolo.x_map(t1_cam_yolo:tend_cam_yolo), lid_pp.y_map(t1_cam_yolo:tend_cam_yolo),'.','markersize',20,'Color',col.camyolo,'displayname','Lid PP');
    if(CAMONLY) scatter(cam_yolo.x_map(t1_cam_yolo:tend_cam_yolo), cam_yolo.y_map(t1_cam_yolo:tend_cam_yolo),20,'filled','DisplayName',NAME_1,'MarkerEdgeColor',col.camyolo,'MarkerFaceColor',col.camyolo); end
    if(compare && CAMONLY) scatter(cam_yolo2.x_map(t1_cam_yolo:tend_cam_yolo), cam_yolo2.y_map(t1_cam_yolo:tend_cam_yolo),20,'filled','DisplayName',NAME_2,'MarkerEdgeColor',col.camyolo2,'MarkerFaceColor',col.camyolo2); end
    if(CAMENH) scatter(cam_yolo_enh.x_map(t1_cam_yolo:tend_cam_yolo), cam_yolo_enh.y_map(t1_cam_yolo:tend_cam_yolo),20,'filled','DisplayName',NAME_1,'MarkerEdgeColor',col.camenh,'MarkerFaceColor',col.camenh); end
    if(compare && CAMEHN) scatter(cam_yolo_enh2.x_map(t1_cam_yolo:tend_cam_yolo), cam_yolo_enh2.y_map(t1_cam_yolo:tend_cam_yolo),20,'filled','DisplayName',NAME_2,'MarkerEdgeColor',col.camenh2,'MarkerFaceColor',col.camenh2); end

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
%     cam_yolo = evalin('base', 'cam_yolo');
%     if(log_tt) tt=evalin('base','tt'); end
%     if(ground_truth); gt =evalin('base','gt'); end
% 
%     t_lim=xlim(axes(1));
%     t1_lid_pp = find(d(lid_pp.sens_stamp>t_lim(1),1);
%     tend_cam_yolo = find(d(lid_pp.sens_stamp<t_lim(2),1,'last');
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
%     % plot(cam_yolo.x_map(t1_lid_pp:tend_cam_yolo), , lid_pp.y_map(t1_lid_pp:tend_cam_yolo),'.','markersize',20,'Color',col.camyolo,'displayname','Lid PP');
%     scatter(cam_yolo.x_map(t1_lid_pp:tend_cam_yolo), , lid_pp.y_map(t1_lid_pp:tend_cam_yolo),20, , lid_pp.score(t1_lid_pp:tend_cam_yolo),'filled','DisplayName','Lidar PP');
%     if(log_tt) plot(tt.x_map(t1_tt:tend_tt),tt.y_map(t1_tt:tend_tt),'Color',col.tt,'DisplayName','tt'); end
%     if(ground_truth)
%         plot(gt.x_map(t1_tt_ref:tend_tt_ref),gt.y_map(t1_tt_ref:tend_tt_ref),'Color',col.ref,'DisplayName','Grond Truth');
%     end
%     legend show
%     colormap turbo
%     colorbar
% end

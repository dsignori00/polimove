clc
close all
clearvars -except log log2 log_ref trajDatabase

compare      = false;
ground_truth = false;

NAME_1 = "";
NAME_2 = "old - ";

%#ok<*UNRCH>
%#ok<*INUSD>

%% LOAD FILES

%load database
if(~exist('trajDatabase','var'))
    trajDatabase = choose_database();
    if(isempty(trajDatabase))
        error('No database selected');
    else
        load(trajDatabase, 'trajDatabase');
    end
end

% load log
if (~exist('log','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    load(fullfile(path,file), 'log');
end

% load log2
if (compare && ~exist('log2','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log 2');
    tmp = load(fullfile(path,file), 'log');
    log2 = tmp.log;
end

if(ground_truth)
    if (~exist('log_ref','var'))
        [file,path] = uigetfile(fullfile(get_gt_dir(),'*.mat'),'Load ground truth log');
        tmp = load(fullfile(path,file), 'out');
        log_ref = tmp.out;
    end
else
    log_ref = [];
end

DateTime = datetime(log.time_offset_nsec,'ConvertFrom','epochtime','TicksPerSecond',1e9,'Format','dd-MMM-yyyy HH:mm:ss');

%% OPTIONS
graphics_options;
col.lidar        = colors.green{2};
col.lidar2       = colors.green{1};
col.radar        = colors.blue{2};
col.radar2       = colors.blue{1};
col.camera       = colors.yellow{2};
col.camera2      = colors.yellow{1};
col.pp = colors.orange{2};
col.pointpillars2= colors.orange{1};
col.ref          = colors.black;

sz=3; % Marker size
f=1;
x_lim = [0 inf];

%% LOAD DATA

[lid_clust, rad_clust, cam_yolo, lid_pp] = load_perception(log);
gt = load_ref(log, false, ground_truth, log_ref);
cam_yolo.sens_stamp(cam_yolo.sens_stamp < 0) = NaN;

if(compare)
    [lid_clust2, rad_clust2, cam_yolo2, lid_pp2] = load_perception(log2);
    cam_yolo2.sens_stamp(cam_yolo2.sens_stamp < 0) = NaN;
    lid_clust2.sens_stamp = lid_clust2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    rad_clust2.sens_stamp = rad_clust2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    cam_yolo2.sens_stamp = cam_yolo2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
    lid_pp2.sens_stamp = lid_pp2.sens_stamp + double(log2.time_offset_nsec-log.time_offset_nsec)*1e-9;
end

%% DETECTIONS FIGURE - RANGE

% Range Computation
lid_clust.range = sqrt(lid_clust.x_rel.^2 + lid_clust.y_rel.^2);
rad_clust.range = sqrt(rad_clust.x_rel.^2 + rad_clust.y_rel.^2);
cam_yolo.range = sqrt(cam_yolo.x_rel.^2 + cam_yolo.y_rel.^2);
lid_pp.range = sqrt(lid_pp.x_rel.^2 + lid_pp.y_rel.^2);

if(compare)
    lid_clust2.range = sqrt(lid_clust2.x_rel.^2 + lid_clust2.y_rel.^2);
    rad_clust2.range = sqrt(rad_clust2.x_rel.^2 + rad_clust2.y_rel.^2);
    cam_yolo2.range = sqrt(cam_yolo2.x_rel.^2 + cam_yolo2.y_rel.^2);
    lid_pp2.range = sqrt(lid_pp2.x_rel.^2 + lid_pp2.y_rel.^2);
end

figure('Name','Detections - Range')
tiledlayout(2,1,'Padding','compact')

% Rho 
axes(f)=nexttile([1,1]); f=f+1; hold on;

scatter(repmat(lid_clust.sens_stamp, size(lid_clust.range,2), 1), lid_clust.range(:), 36, col.lidar, '*', 'DisplayName', NAME_1+"lidar");
scatter(repmat(rad_clust.sens_stamp, size(rad_clust.range,2), 1), rad_clust.range(:), 36, col.radar, '*', 'DisplayName', NAME_1+"radar");
scatter(repmat(lid_pp.sens_stamp, size(lid_pp.range,2), 1), lid_pp.range(:), 36, col.pp, '*', 'DisplayName', NAME_1+"pointpillars");
scatter(repmat(cam_yolo.sens_stamp, size(cam_yolo.range,2), 1), cam_yolo.range(:), 36, col.camera, '*', 'DisplayName', NAME_1+"yolo");
if(compare)
    scatter(repmat(lid_clust2.sens_stamp,size(lid_clust2.range,2), 1)', lid_clust2.range(:), 36, col.lidar2, 'filled', 'DisplayName', NAME_2+"lidar");
    scatter(repmat(rad_clust2.sens_stamp,size(rad_clust2.range,2), 1)', rad_clust2.range(:), 36, col.radar2, 'filled', 'DisplayName', NAME_2+"radar");
    scatter(repmat(lid_pp2.sens_stamp,size(lid_pp2.range,2), 1)', lid_pp2.range(:), 36, col.pointpillars2, 'filled', 'DisplayName', NAME_2+"pointpillars");
    scatter(repmat(cam_yolo2.sens_stamp,size(cam_yolo2.range,2), 1)', cam_yolo2.range(:), 36, col.camera2, 'filled', 'DisplayName', NAME_2+"yolo");
end
if(ground_truth); plot(gt.stamp, gt.rho, 'Color', col.ref, 'DisplayName', 'ground truth'); end
title("range [m]"); xlim(x_lim); ylim([0 200]); grid on; xlabel('time [s]'); ylabel('range [m]'); legend;


% Rho Dot 
axes(f) = nexttile([1,1]); f=f+1; hold on;
scatter(repmat(rad_clust.sens_stamp, size(rad_clust.rho_dot,2), 1), rad_clust.rho_dot(:), 36, 'd', 'MarkerEdgeColor', col.radar, 'DisplayName', NAME_1+"radar");
if(compare); scatter(repmat(rad_clust2.sens_stamp, size(rad_clust2.rho_dot,2), 1), rad_clust2.rho_dot(:), 36, 'd', 'MarkerEdgeColor', col.radar2, 'DisplayName', NAME_2+" - radar"); end
if(ground_truth); plot(gt.stamp, gt.rho_dot, 'Color', col.ref, 'DisplayName', 'ground truth'); end
title("range rate [m/s]"); xlim(x_lim); grid on; xlabel('time [s]'); ylabel('range rate [m/s]'); legend;

linkaxes(axes,'x');

%% FOV

[fov.cam_x,fov.cam_y]  = compute_fov(85,0);
[fov.lid_x,fov.lid_y]  = compute_fov(10,179);
[fov.rad_x,fov.rad_y]  = compute_fov(90,0);

[range.x_25m, range.y_25m] = calculate_ellipse(25,25,0,0); 
[range.x_50m, range.y_50m] = calculate_ellipse(50,50,0,0); 
[range.x_75m, range.y_75m] = calculate_ellipse(75,75,0,0); 
[range.x_100m, range.y_100m] = calculate_ellipse(100,100,0,0); 

fov_fig = figure('name','FOV');

l = uicontrol('Style','pushbutton');
l.String = {'Refresh'};
l.Callback = @fovButtonPushed;

function fovButtonPushed(~,~)
    axes = evalin('base', 'axes');
    col = evalin('base', 'col');
    lid_clust = evalin('base', 'lid_clust');
    rad_clust = evalin('base', 'rad_clust');
    cam_yolo = evalin('base', 'cam_yolo');
    lid_pp = evalin('base', 'lid_pp');
    fov = evalin('base', 'fov');
    range = evalin('base', 'range');
    compare = evalin('base', 'compare');
    NAME_1 = evalin('base', 'NAME_1');
    NAME_2 = evalin('base', 'NAME_2');
    if(compare)
        lid_clust2 = evalin('base', 'lid_clust2');
        rad_clust2 = evalin('base', 'rad_clust2');
        cam_yolo2 = evalin('base', 'cam_yolo2');
        lid_pp2 = evalin('base', 'lid_pp2');
    end
  
    t_lim=xlim(axes(1));
    t1_lid_clust = find(lid_clust.sens_stamp>t_lim(1),1);
    tend_lid_clust = find(lid_clust.sens_stamp<t_lim(2),1,'last');
    t1_rad_clust = find(rad_clust.sens_stamp>t_lim(1),1);
    tend_rad_clust = find(rad_clust.sens_stamp<t_lim(2),1,'last');
    t1_cam_yolo = find(cam_yolo.sens_stamp>t_lim(1),1);
    tend_cam_yolo = find(cam_yolo.sens_stamp<t_lim(2),1,'last');
    t1_lid_pp = find(lid_pp.sens_stamp>t_lim(1),1);
    tend_lid_pp = find(lid_pp.sens_stamp<t_lim(2),1,'last');

    if(compare)
        t1_lid_clust2 = find(lid_clust2.sens_stamp>t_lim(1),1);
        tend_lid_clust2 = find(lid_clust2.sens_stamp<t_lim(2),1,'last');
        t1_rad_clust2 = find(rad_clust2.sens_stamp>t_lim(1),1);
        tend_rad_clust2 = find(rad_clust2.sens_stamp<t_lim(2),1,'last');
        t1_cam_yolo2 = find(cam_yolo2.sens_stamp>t_lim(1),1);
        tend_cam_yolo2 = find(cam_yolo2.sens_stamp<t_lim(2),1,'last');
        t1_lid_pp2 = find(lid_pp2.sens_stamp>t_lim(1),1);
        tend_lid_pp2 = find(lid_pp2.sens_stamp<t_lim(2),1,'last');
    end

    % Sensor Plot
    subplot(2,2,[1 3])
    cla reset 
    legend('Location','northwest')
    hold on
    grid on
    xlabel('y[m]')
    ylabel('x[m]')
    axis equal
    set(gca,'XDir','reverse')

    plot(cam_yolo.y_rel(t1_cam_yolo:tend_cam_yolo),cam_yolo.x_rel(t1_cam_yolo:tend_cam_yolo),'*','Color',col.camera,'DisplayName',NAME_1+"yolo")
    plot(rad_clust.y_rel(t1_rad_clust:tend_rad_clust),rad_clust.x_rel(t1_rad_clust:tend_rad_clust),'*','Color',col.radar,'DisplayName',NAME_1+"radar")
    plot(lid_pp.y_rel(t1_lid_pp:tend_lid_pp),lid_pp.x_rel(t1_lid_pp:tend_lid_pp),'*','Color',col.pp,'DisplayName',NAME_1+"pointpillars")
    plot(lid_clust.y_rel(t1_lid_clust:tend_lid_clust),lid_clust.x_rel(t1_lid_clust:tend_lid_clust),'*','Color',col.lidar,'DisplayName',NAME_1+"lidar")
    if(compare)
        plot(cam_yolo2.y_rel(t1_cam_yolo2:tend_cam_yolo2),cam_yolo2.x_rel(t1_cam_yolo2:tend_cam_yolo2),'*','Color',col.camera2,'DisplayName',NAME_2+"yolo")
        plot(rad_clust2.y_rel(t1_rad_clust2:tend_rad_clust2),rad_clust2.x_rel(t1_rad_clust2:tend_rad_clust2),'*','Color',col.radar2,'DisplayName',NAME_2+"radar")
        plot(lid_pp2.y_rel(t1_lid_pp2:tend_lid_pp2),lid_pp2.x_rel(t1_lid_pp2:tend_lid_pp2),'*','Color',col.pointpillars2,'DisplayName',NAME_2+"pointpillars")
        plot(lid_clust2.y_rel(t1_lid_clust2:tend_lid_clust2),lid_clust2.x_rel(t1_lid_clust2:tend_lid_clust2),'*','Color',col.lidar2,'DisplayName',NAME_2+"lidar")
    end
    title("sensors fov")
    plot(fov.rad_y,fov.rad_x,'-','LineWidth',0.5,'Color',col.radar,'HandleVisibility','off')
    plot(fov.rad_y,-fov.rad_x,'-','LineWidth',0.5,'Color',col.radar,'HandleVisibility','off')
    plot(fov.lid_y,fov.lid_x,'-','LineWidth',0.5,'Color',col.lidar,'HandleVisibility','off')
    plot(fov.cam_y,fov.cam_x,'-','LineWidth',0.5,'Color',col.camera,'HandleVisibility','off')
    plot(fov.cam_y,-fov.cam_x,'-','LineWidth',0.5,'Color',col.camera,'HandleVisibility','off')

    plot(range.x_25m,range.y_25m,'--','LineWidth',0.3,'Color',[0.5 0.5 0.5],'HandleVisibility','off')
    plot(range.x_50m,range.y_50m,'--','LineWidth',0.3,'Color',[0.5 0.5 0.5],'HandleVisibility','off')
    plot(range.x_75m,range.y_75m,'--','LineWidth',0.3,'Color',[0.5 0.5 0.5],'HandleVisibility','off')
    plot(range.x_100m,range.y_100m,'--','LineWidth',0.3,'Color',[0.5 0.5 0.5],'HandleVisibility','off')
    plot(0,0,'d','Color','default','LineWidth',1,'DisplayName','Ego','MarkerSize',10)
    xlim([-125 125])

    % Normalized Range distribution
    rangeAxes(1) = subplot(2,2,2);
    cla reset,hold on, grid on
    xlabel('range [m]')
    ylabel('normalized counts')
    title(NAME_1)

    histogram(lid_clust.range(t1_lid_clust:tend_lid_clust), 'Normalization','probability','BinWidth',1,'FaceColor',col.lidar,'DisplayName',"lidar");
    histogram(rad_clust.range(t1_rad_clust:tend_rad_clust), 'Normalization','probability','BinWidth',1,'FaceColor',col.radar,'DisplayName',"radar");
    histogram(lid_pp.range(t1_lid_pp:tend_lid_pp), 'Normalization','probability','BinWidth',1,'FaceColor',col.pp,'DisplayName',"pointpillars");
    histogram(cam_yolo.range(t1_cam_yolo:tend_cam_yolo), 'Normalization','probability','BinWidth',1,'FaceColor',col.camera,'DisplayName',"yolo");
    
    if(compare)
        rangeAxes(2) = subplot(2,2,4);
        cla reset,hold on, grid on
        xlabel('range [m]')
        ylabel('normalized counts')
        title(NAME_2)
        histogram(lid_clust2.range(t1_lid_clust2:tend_lid_clust2), 'Normalization','probability','BinWidth',1,'FaceColor',col.lidar2,'DisplayName',"lidar");
        histogram(rad_clust2.range(t1_rad_clust2:tend_rad_clust2), 'Normalization','probability','BinWidth',1,'FaceColor',col.radar2,'DisplayName',"radar");
        histogram(lid_pp2.range(t1_lid_pp2:tend_lid_pp2), 'Normalization','probability','BinWidth',1,'FaceColor',col.pointpillars2,'DisplayName',"pointpillars");
        histogram(cam_yolo2.range(t1_cam_yolo2:tend_cam_yolo2), 'Normalization','probability','BinWidth',1,'FaceColor',col.camera2,'DisplayName',"yolo");

        linkaxes(rangeAxes,'x');
    end

    xlim(rangeAxes,[0 150]);
    ylim(rangeAxes,[0 0.005]);

end

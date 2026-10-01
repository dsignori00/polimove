%% Covariance Analysis

% IDEAS:
% - only consider the first track
% - only plot associated measurements
% - usa bag simulate con bag reader: pubblico stato filtro dopo ciascuna misurazione 
%   (attenzione a effetto reprocessing -> vedi latenza, aggiungi campo a oppoenents)
% - plot covariance with shaded area (2 sigma) (vedi plot del cingo -> utile per analisi associazione misure)


if ~isfield(tt, 'measures')
    warning('tt struct must contain measures field for covariance analysis (resimulate using bag reader).');
    return;
end

if(opp_idx > tt.max_opp)
    warning('Selected opp_idx exceeds the number of tracked opponents. Using first opponent instead.');
    opp_idx = 1;  
end

% gating distance
dist = 5;  % [m]

% measurement matrices
R.lidar         = diag([0.7 0.7]);   % [m^2]
R.radar         = diag([1.5 2.0]);   % [m^2]
R.camera        = diag([10.0 10.0]); % [m^2]
R.pointpillars  = diag([0.7 0.7]);   % [m^2]

sensor_list = { ...
    'lidar',  SensorType.LIDAR_CLUSTERING,   col.lidar,        R.lidar,         lid_clust;
    'radar',  SensorType.RADAR_CLUSTERING,   col.radar,        R.radar,         rad_clust;
    'pp',     SensorType.LIDAR_POINTPILLARS, col.pp, R.pointpillars,  lid_pp;
    'camera', SensorType.CAMERA_YOLO,        col.camera,       R.camera,        cam_yolo;
};

%% Time series figure


figure('name', 'Covariance - Positions', 'NumberTitle', 'off');
tiledlayout(3,1,'Padding','compact');
sensor_points = struct();

source_vec = reshape(tt.measures.source(:,opp_idx,:), [], 1);   % [N*M x 1]
stamp_vec  = reshape(tt.measures.stamp(:,opp_idx,:),  [], 1);   % same size as source_vec

[unique_stamps, ia] = unique(tt.stamp);

yaw = deg2rad(tt.yaw_map(:,opp_idx));
cy = cos(yaw);
sy = sin(yaw);
cov_xx_map = squeeze(tt.covariance(:,opp_idx,1));
cov_xy_map = squeeze(tt.covariance(:,opp_idx,2));
cov_yx_map = squeeze(tt.covariance(:,opp_idx,7));
cov_yy_map = squeeze(tt.covariance(:,opp_idx,8));
cov_xy_sum = cov_xy_map + cov_yx_map;

cov_x_cog = sqrt(cy.^2 .* cov_xx_map + cy .* sy .* cov_xy_sum + sy.^2 .* cov_yy_map);
cov_y_cog = sqrt(sy.^2 .* cov_xx_map - cy .* sy .* cov_xy_sum + cy.^2 .* cov_yy_map);

ax(f) = nexttile([1,1]); f=f+1; hold on;
% --- Loop over each sensor type ---
for k = 1:size(sensor_list,1)
    name = sensor_list{k,1};
    type = sensor_list{k,2};
    color = sensor_list{k,3};
    noise = sensor_list{k,4};
    
    idx = (source_vec == type.Value);   % logical vector
    unique_cov = cov_x_cog(ia);
    sensor_points.(name) = interp1(unique_stamps, unique_cov, stamp_vec(idx));

    plot(stamp_vec(idx), sensor_points.(name), 'o', ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', color, ...
        'MarkerSize', sz + size(sensor_list,1) - k, ...
        'DisplayName', ['R: ' sprintf('%.1f', noise(1,1)) ' - ' strrep(name,'_',' ')]); 
    hold on
end
plot(tt.stamp, cov_x_cog,'Color', col.tt, 'DisplayName', 'track');
grid on; ylabel('x cog [m]'); legend show;


ax(f) = nexttile([1,1]); f=f+1; hold on;
% --- Loop over each sensor type ---
for k = 1:size(sensor_list,1)
    name = sensor_list{k,1};
    type = sensor_list{k,2};
    color = sensor_list{k,3};
    noise = sensor_list{k,4};
    
    idx = (source_vec == type.Value);   % logical vector
    unique_cov = cov_y_cog(ia);
    sensor_points.(name) = interp1(unique_stamps, unique_cov, stamp_vec(idx));

    plot(stamp_vec(idx), sensor_points.(name), 'o', ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', color, ...
        'MarkerSize', sz + size(sensor_list,1) - k, ...
        'DisplayName', ['R: ' sprintf('%.1f', noise(2,2)) ' - ' strrep(name,'_',' ')]); 
    hold on
end
plot(tt.stamp,cov_y_cog,'Color',col.tt,'DisplayName','track');
grid on; ylabel('y cog [m]');  legend show;


% yaw
ax(f) = nexttile([1,1]); f=f+1; hold on;
% --- Loop over each sensor type ---
for k = 1:size(sensor_list,1)
    name = sensor_list{k,1};
    type = sensor_list{k,2};
    color = sensor_list{k,3};
    
    idx = (source_vec == type.Value);   % logical vector
    unique_cov = sqrt(tt.covariance(ia,opp_idx,22));
    sensor_points.(name) = interp1(unique_stamps, unique_cov, stamp_vec(idx));

    plot(stamp_vec(idx), rad2deg(sensor_points.(name)), 'o', ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', color, ...
        'MarkerSize', sz + size(sensor_list,1) - k, ...
        'DisplayName', strrep(name,'_',' '));  % nicer display
    hold on
end
plot(tt.stamp,rad2deg(sqrt(tt.covariance(:,opp_idx, 22))),'Color',col.tt,'DisplayName','track');
grid on; ylabel('yaw map [deg]'); legend show; xlabel('timestamp [s]');


figure('name', 'Covariance - Speed Acc', 'NumberTitle', 'off');
tiledlayout(3,1,'Padding','compact');

% speed
ax(f) = nexttile([1,1]); f=f+1; hold on;
% --- Loop over each sensor type ---
for k = 1:size(sensor_list,1)
    name = sensor_list{k,1};
    type = sensor_list{k,2};
    color = sensor_list{k,3};
    
    idx = (source_vec == type.Value);   % logical vector
    unique_cov = sqrt(tt.covariance(ia,opp_idx,15));
    sensor_points.(name) = interp1(unique_stamps, unique_cov, stamp_vec(idx));

    plot(stamp_vec(idx), sensor_points.(name), 'o', ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', color, ...
        'MarkerSize', sz + size(sensor_list,1) - k, ...
        'DisplayName', strrep(name,'_',' '));  % nicer display
    hold on
end
plot(tt.stamp,sqrt(tt.covariance(:,opp_idx, 15)),'Color',col.tt,'DisplayName','track');
grid on; ylabel('speed [m/s]'); legend show;


% acceleration
ax(f) = nexttile([1,1]); f=f+1; hold on;
% --- Loop over each sensor type ---
for k = 1:size(sensor_list,1)
    name = sensor_list{k,1};
    type = sensor_list{k,2};
    color = sensor_list{k,3};
    
    idx = (source_vec == type.Value);   % logical vector
    unique_cov = sqrt(tt.covariance(ia,opp_idx,36));
    sensor_points.(name) = interp1(unique_stamps, unique_cov, stamp_vec(idx));

    plot(stamp_vec(idx), sensor_points.(name), 'o', ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', color, ...
        'MarkerSize', sz + size(sensor_list,1) - k, ...
        'DisplayName', strrep(name,'_',' '));  % nicer display
    hold on
end
plot(tt.stamp,sqrt(tt.covariance(:,opp_idx, 36)),'Color',col.tt,'DisplayName','track');
grid on; ylabel('acc [m/s$^2$]'); legend show; 

% yaw rate
ax(f) = nexttile([1,1]); f=f+1; hold on;
% --- Loop over each sensor type ---
for k = 1:size(sensor_list,1)
    name = sensor_list{k,1};
    type = sensor_list{k,2};
    color = sensor_list{k,3};
    
    idx = (source_vec == type.Value);   % logical vector
    unique_cov = sqrt(tt.covariance(ia,opp_idx,29));
    sensor_points.(name) = interp1(unique_stamps, unique_cov, stamp_vec(idx));

    plot(stamp_vec(idx), rad2deg(sensor_points.(name)), 'o', ...
        'MarkerFaceColor', color, ...
        'MarkerEdgeColor', color, ...
        'MarkerSize', sz + size(sensor_list,1) - k, ...
        'DisplayName', strrep(name,'_',' '));  % nicer display
    hold on
end
plot(tt.stamp,rad2deg(sqrt(tt.covariance(:,opp_idx, 29))),'Color',col.tt,'DisplayName','track');
grid on; ylabel('yaw rate [deg/s]'); legend show;xlabel('timestamp [s]');


%% Association Figure


%%% Select time portion on any plots, click refresh, use arrows to show
%%% each iteration in the selected range
assFig = figure('name', 'Covariance - Association', 'NumberTitle', 'off');
set(assFig,'KeyPressFcn',@keyPressed);   

% Axes for map (left side)
axMap = builtin('axes','Parent',assFig);
title(axMap,'map');

% Button
c = c + 1;
b(c) = uicontrol('Style','pushbutton', ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@refreshTimeButtonPushed);

% Initialize empty state
S = struct();
S.ax       = axMap;
guidata(assFig,S);

function refreshTimeButtonPushed(~,~)
    % Pull needed vars
    assFig     = evalin('base', 'assFig');
    ax          = evalin('base', 'ax');
    tt         = evalin('base', 'tt');
    traj_db    = evalin('base', 'trajDatabase');
    col        = evalin('base', 'col');
    dist       = evalin('base', 'dist');

    % Determine range from x-limits
    t_lim = xlim(ax(1));
    t1_tt   = find(tt.stamp > t_lim(1), 1, 'first');
    tend_tt = find(tt.stamp < t_lim(2), 1, 'last');

    if isempty(t1_tt) || isempty(tend_tt) || t1_tt > tend_tt
        warning('No opponents samples in the selected x-limits interval.');
        return;
    end

    % Build/update state
    S = guidata(assFig);

    S.dist      = dist;
    S.col       = col;
    S.traj_db   = traj_db;
    S.tt        = tt;
    S.iStart    = t1_tt;
    S.iEnd      = tend_tt;
    S.iCur      = t1_tt;

    guidata(assFig,S);

    drawCurrentSample();
end

function keyPressed(~, event)
    assFig = evalin('base','assFig');
    S = guidata(assFig);

    if ~isfield(S,'iCur') || isempty(S.iCur)
        return;
    end

    switch event.Key
        case 'rightarrow'
            S.iCur = min(S.iCur + 1, S.iEnd);
            guidata(assFig,S);
            drawCurrentSample();

        case 'leftarrow'
            S.iCur = max(S.iCur - 1, S.iStart);
            guidata(assFig,S);
            drawCurrentSample();
    end
end

function drawCurrentSample()
    assFig = evalin('base','assFig');
    S = guidata(assFig);
    sensor_list = evalin('base', 'sensor_list');
    i = S.iCur;

    % --------- MAP DRAW ----------
    builtin('axes', S.ax);
    cla(S.ax,'reset');
    hold(S.ax,'on'); grid(S.ax,'on');
    xlabel(S.ax,'x[m]'); ylabel(S.ax,'y[m]'); axis(S.ax,'equal'); 

    % stampa tutte la misura tra le due iterazioni i e i-1 (solo 1, usa tt simulator)
    query_time = [];
    for k = 1:size(sensor_list,1)
        name  = sensor_list{k,1};
        color = sensor_list{k,3};
        noise = sensor_list{k,4};
        data  = sensor_list{k,5};

        if i == 1
            continue;
        end

        idx = find(data.stamp >= S.tt.stamp(i-1) & data.stamp < S.tt.stamp(i));
        if ~any(idx)
            continue;
        end

        query_time = data.sens_stamp(idx);

        for j = 1:sum(~isnan(data.x_map(idx,:)))
            % plot covariance ellipse
            mu = [data.x_map(idx,j); 
                  data.y_map(idx,j)];

            % rotazione yaw - per essere 'precisi' dovrei interpolare la yaw del veicolo al tempo della misura
            yaw = deg2rad(S.tt.yaw_map(i,j));
            if isnan(yaw)
                continue
            else
                R_vehicle_world = [cos(yaw), -sin(yaw);
                                   sin(yaw),  cos(yaw)];
            end

            noise_map = R_vehicle_world * noise * R_vehicle_world';
            [xe, ye] = covariance_ellipse(noise_map, mu, 1);  % ellisse 1-sigma
    
            patch(xe, ye, color, ...
                'FaceAlpha', 0.3, ...
                'EdgeColor', color, ...
                'LineWidth', 1, ...
                'HandleVisibility','off');
        end

        scatter(data.x_map(idx,:), data.y_map(idx,:), 'filled', ...
            'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, ...
            'DisplayName', name);
    end

    [unique_stamps, ia] = unique(S.tt.stamp);
    unique_x = S.tt.x_map(ia,:);
    unique_y = S.tt.y_map(ia,:);
    x = interp1(unique_stamps, unique_x, query_time);
    y = interp1(unique_stamps, unique_y, query_time);
    for j = 1:length(~isnan(x))
        if isnan(x(j)) || isnan(y(j))
            continue;
        end

        % euclidean distance 
        [xt, yt] = covariance_ellipse(diag([S.dist.^2, S.dist.^2]), [x(j); y(j)], 1);  % ellisse 1-sigma
        patch(xt, yt, S.col.tt, ...
            'FaceAlpha', 0.0, ...
            'EdgeColor', S.col.tt, ...
            'LineWidth', 1, ...
            'LineStyle', '--', ...
            'HandleVisibility','off');
        scatter(x(j), y(j), 'filled', 'MarkerFaceColor',S.col.tt, 'DisplayName','track');
        
        % state covariance - NB: approximation, should predict back to measure time
        sigma = reshape(S.tt.covariance(i,j,:), 6, 6);
        [xc, yc] = covariance_ellipse(sigma(1:2,1:2), [x(j); y(j)], 1);  % ellisse 1-sigma
        patch(xc, yc, S.col.tt, ...
            'FaceAlpha', 0.3, ...
            'EdgeColor', S.col.tt, ...
            'LineWidth', 1, ...
            'HandleVisibility','off');
    end

    margin = 20;
    if ~isnan(S.tt.x_map(i,1)) && ~isnan(S.tt.y_map(i,1))
        xlim(S.ax, [min([S.tt.x_map(i,:)]) - margin max([S.tt.x_map(i,:)]) + margin]);
        ylim(S.ax, [min([S.tt.y_map(i,:)]) - margin max([S.tt.y_map(i,:)]) + margin]);
        axis(S.ax, 'manual');
    end

    % plot track lines
    id_left = length(S.traj_db) - 2;
    id_right = length(S.traj_db) - 1;
    plot(S.traj_db(id_left).X, S.traj_db(id_left).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
    plot(S.traj_db(id_right).X, S.traj_db(id_right).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');

    txt = sprintf('sample %d / %d', i-S.iStart+1, S.iEnd-S.iStart+1);
    title(S.ax, ['map - ' txt]); legend(S.ax,'show');
    hold(S.ax,'off');
end

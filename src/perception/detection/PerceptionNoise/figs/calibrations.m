%% CALIBRATION_ERRORS
% Per sensor in sensors{}:
%   lateral:      ey = TY + tan(dpsi) * x_rel, fitted separately by x_rel sign
%   longitudinal: ex = TX, evaluated separately for x_rel > 0 and x_rel < 0

if ~exist('err_thr','var'), err_thr = 10; end
MAX_GAP_S = 0.2;   % max distance from nearest GT sample [s]
STIFFNESS_MIN_RANGE_M = 15;

% Ground truth
[t_gt,iu] = unique(double(gt.stamp(:)));
x_gt = double(gt.x_rel(iu));
y_gt = double(gt.y_rel(iu));
v = isfinite(t_gt) & isfinite(x_gt) & isfinite(y_gt);
t_gt = t_gt(v); x_gt = x_gt(v); y_gt = y_gt(v);

calibration_fig = figure('Name','Extrinsic errors', ...
    'NumberTitle','off','Color','w');
calibration_tabs = uitabgroup(calibration_fig);

for i = 1:numel(sensors)
    S  = sensors{i}.s;
    tq = double(S.sens_stamp(:));
    x  = double(S.x_rel(:));

    x_ref = interp1(t_gt,x_gt,tq,'linear',NaN);
    y_ref = interp1(t_gt,y_gt,tq,'linear',NaN);
    ex  = x - x_ref;
    ey  = double(S.y_rel(:)) - y_ref;
    gap = abs(tq - interp1(t_gt,t_gt,tq,'nearest',NaN));

    ok = gap <= MAX_GAP_S & abs(ex) < err_thr & abs(ey) < err_thr;
    [~,u] = unique(round(tq*1e3));            % remove duplicated scans
    ok = ok & ismember((1:numel(tq))',u);

    if nnz(ok) < 2
        warning('PerceptionNoise:InsufficientCalibrationSamples', ...
            '%s has fewer than two valid calibration samples and was skipped.', ...
            sensors{i}.name);
        continue
    end

    pos_x = ok & x > 0;
    neg_x = ok & x < 0;
    TX   = mean(ex(ok));
    TX_pos = mean(ex(pos_x));
    TX_neg = mean(ex(neg_x));

    p_pos = [NaN NaN];
    dpsi_pos = NaN;
    R2_pos = NaN;
    if nnz(pos_x) >= 2
        p_pos = polyfit(x(pos_x),ey(pos_x),1);
        dpsi_pos = atand(p_pos(1));
        ey_fit_pos = polyval(p_pos,x(pos_x));
        ss_res_pos = sum((ey(pos_x) - ey_fit_pos).^2);
        ss_tot_pos = sum((ey(pos_x) - mean(ey(pos_x))).^2);
        if ss_tot_pos > 0
            R2_pos = 1 - ss_res_pos/ss_tot_pos;
        end
    end

    p_neg = [NaN NaN];
    dpsi_neg = NaN;
    R2_neg = NaN;
    if nnz(neg_x) >= 2
        p_neg = polyfit(x(neg_x),ey(neg_x),1);
        dpsi_neg = atand(p_neg(1));
        ey_fit_neg = polyval(p_neg,x(neg_x));
        ss_res_neg = sum((ey(neg_x) - ey_fit_neg).^2);
        ss_tot_neg = sum((ey(neg_x) - mean(ey(neg_x))).^2);
        if ss_tot_neg > 0
            R2_neg = 1 - ss_res_neg/ss_tot_neg;
        end
    end
    fprintf(['%-8s  N = %5d | TX = %+.3f m | ' ...
        'TX(x_rel > 0) = %+.3f m (N = %d) | ' ...
        'TX(x_rel < 0) = %+.3f m (N = %d)\n'], ...
        sensors{i}.name,nnz(ok),TX, ...
        TX_pos,nnz(pos_x),TX_neg,nnz(neg_x));
    fprintf(['          Lateral x_rel > 0: TY = %+.3f m | ' ...
        'dpsi = %+.3f deg | R2 = %.3f (N = %d)\n'], ...
        p_pos(2),dpsi_pos,R2_pos,nnz(pos_x));
    fprintf(['          Lateral x_rel < 0: TY = %+.3f m | ' ...
        'dpsi = %+.3f deg | R2 = %.3f (N = %d)\n'], ...
        p_neg(2),dpsi_neg,R2_neg,nnz(neg_x));

    sensor_tab = uitab(calibration_tabs,'Title',sensors{i}.name);
    if strcmp(sensors{i}.id,'rad_clust')
        radar_tabs = uitabgroup(sensor_tab);
        extrinsic_tab = uitab(radar_tabs,'Title','Extrinsic');
        stiffness_tab = uitab(radar_tabs,'Title','Stiffness');
        layout_parent = extrinsic_tab;
    else
        layout_parent = sensor_tab;
    end
    sensor_layout = tiledlayout(layout_parent,2,1, ...
        'Padding','compact','TileSpacing','compact');

    % Lateral error vs x
    lateral_ax = nexttile(sensor_layout);
    hold on; grid on; box on
    plot(x(ok),ey(ok),'.', ...
        'Color',sensors{i}.col, ...
        'MarkerSize',6, ...
        'DisplayName','Measurement error');
    if nnz(pos_x) >= 2
        xf_pos = linspace(min(x(pos_x)),max(x(pos_x)),2);
        fit_label_pos = sprintf(['Fit ($x_{rel}>0$): ' ...
            '$T_Y = %.3f\\,\\mathrm{m},\\;\\Delta\\psi = %.2f^{\\circ},\\;' ...
            'R^2 = %.3f$'],p_pos(2),dpsi_pos,R2_pos);
        plot(xf_pos,polyval(p_pos,xf_pos),'-', ...
            'Color',[0 0.4470 0.7410], ...
            'LineWidth',2, ...
            'DisplayName',fit_label_pos);
    end
    if nnz(neg_x) >= 2
        xf_neg = linspace(min(x(neg_x)),max(x(neg_x)),2);
        fit_label_neg = sprintf(['Fit ($x_{rel}<0$): ' ...
            '$T_Y = %.3f\\,\\mathrm{m},\\;\\Delta\\psi = %.2f^{\\circ},\\;' ...
            'R^2 = %.3f$'],p_neg(2),dpsi_neg,R2_neg);
        plot(xf_neg,polyval(p_neg,xf_neg),'-', ...
            'Color',[0.8500 0.3250 0.0980], ...
            'LineWidth',2, ...
            'DisplayName',fit_label_neg);
    end
    yline(0,'--k','HandleVisibility','off');
    xline(0,'--k','HandleVisibility','off');
    ylim([-1 1]); ylabel('y error [m]'); title([sensors{i}.name ' - lateral']);
    legend('show','Location','northwest');
    xlabel('x relative [m]');

    % Longitudinal error vs x
    longitudinal_ax = nexttile(sensor_layout);
    hold on; grid on; box on
    plot(x(ok),ex(ok),'.', ...
        'Color',sensors{i}.col, ...
        'MarkerSize',6, ...
        'DisplayName','Measurement error');
    if any(pos_x)
        pos_mean_label = sprintf( ...
            'Mean ($x_{rel}>0$): $T_X = %.3f\\,\\mathrm{m}$',TX_pos);
        plot([min(x(pos_x)) max(x(pos_x))],[TX_pos TX_pos],'-', ...
            'Color',[0 0.4470 0.7410], ...
            'LineWidth',2, ...
            'DisplayName',pos_mean_label);
    end
    if any(neg_x)
        neg_mean_label = sprintf( ...
            'Mean ($x_{rel}<0$): $T_X = %.3f\\,\\mathrm{m}$',TX_neg);
        plot([min(x(neg_x)) max(x(neg_x))],[TX_neg TX_neg],'-', ...
            'Color',[0.8500 0.3250 0.0980], ...
            'LineWidth',2, ...
            'DisplayName',neg_mean_label);
    end
    yline(0,'--k','HandleVisibility','off');
    xline(0,'--k','HandleVisibility','off');
    ylim([-1 1]); ylabel('x error [m]'); title([sensors{i}.name ' - longitudinal']);
    legend('show','Location','northwest');
    xlabel('x relative [m]');

    linkaxes([lateral_ax longitudinal_ax],'xy');

    if strcmp(sensors{i}.id,'rad_clust')
        plotRadarStiffness(stiffness_tab,S,tq,x,x_ref,y_ref,ok, ...
            log,sensors{i}.col,MAX_GAP_S,STIFFNESS_MIN_RANGE_M);
    end
end

function plotRadarStiffness(parent_tab,sensor,tq,x_rel,x_ref,y_ref, ...
        calibration_mask,log,sensor_color,max_gap_s,min_range_m)
%PLOTRADARSTIFFNESS Plot bearing sensitivity to ego lateral acceleration.

ego_stamp = double(log.estimation.stamp__tot(:));
ego_lateral_acceleration = double(log.estimation.ay(:))/9.80665;
valid_ego = isfinite(ego_stamp) & isfinite(ego_lateral_acceleration);
ego_stamp = ego_stamp(valid_ego);
ego_lateral_acceleration = ego_lateral_acceleration(valid_ego);
[ego_stamp,unique_indices] = unique(ego_stamp,'sorted');
ego_lateral_acceleration = ego_lateral_acceleration(unique_indices);

if numel(ego_stamp) < 2
    warning('PerceptionNoise:InsufficientLateralAccelerationSamples', ...
        'Radar stiffness needs at least two lateral-acceleration samples.');
    return;
end

lateral_acceleration = interp1( ...
    ego_stamp,ego_lateral_acceleration,tq,'linear',NaN);
nearest_ego_stamp = interp1(ego_stamp,ego_stamp,tq,'nearest',NaN);

measured_bearing = atan2d(double(sensor.y_rel(:)),x_rel);
reference_bearing = atan2d(y_ref,x_ref);
bearing_difference = measured_bearing - reference_bearing;
bearing_error = atan2d(sind(bearing_difference),cosd(bearing_difference));

valid = calibration_mask & ...
    isfinite(lateral_acceleration) & isfinite(bearing_error) & ...
    hypot(x_rel,double(sensor.y_rel(:))) >= min_range_m & ...
    abs(tq - nearest_ego_stamp) <= max_gap_s;
front = valid & x_rel > 0;
back = valid & x_rel < 0;

stiffness_layout = tiledlayout(parent_tab,2,1, ...
    'Padding','compact','TileSpacing','compact');
front_ax = nexttile(stiffness_layout);
front_model = plotStiffnessDirection( ...
    front_ax,lateral_acceleration(front),bearing_error(front),sensor_color, ...
    [0 0.4470 0.7410],'x_{rel}>0','Front radar');

back_ax = nexttile(stiffness_layout);
back_model = plotStiffnessDirection( ...
    back_ax,lateral_acceleration(back),bearing_error(back),sensor_color, ...
    [0.8500 0.3250 0.0980],'x_{rel}<0','Back radar');

xlabel(back_ax,'ego lateral acceleration [g]');
linkaxes([front_ax back_ax],'xy');

fprintf(['radar     Stiffness x_rel > 0: theta0 = %+.3f deg | ' ...
    'Ca = %+.4f deg/g | R2 = %.3f (N = %d)\n'], ...
    front_model.offset,front_model.slope,front_model.r_squared, ...
    front_model.samples);
fprintf(['          Stiffness x_rel < 0: theta0 = %+.3f deg | ' ...
    'Ca = %+.4f deg/g | R2 = %.3f (N = %d)\n'], ...
    back_model.offset,back_model.slope,back_model.r_squared, ...
    back_model.samples);
end

function model = plotStiffnessDirection(ax,lateral_acceleration,bearing_error, ...
        sensor_color,fit_color,direction_condition,direction_title)
%PLOTSTIFFNESSDIRECTION Plot one radar direction and its linear fit.

hold(ax,'on');
grid(ax,'on');
box(ax,'on');

plot(ax,lateral_acceleration,bearing_error,'.', ...
    'Color',sensor_color, ...
    'MarkerSize',6, ...
    'DisplayName','Measurement error');

model = fitLinearSensitivity(lateral_acceleration,bearing_error);
if model.valid
    acceleration_fit = linspace( ...
        min(lateral_acceleration),max(lateral_acceleration),100);
    fit_label = sprintf(['Fit ($%s$): ' ...
        '$\\theta_0 = %+.3f^{\\circ},\\;C_a = ' ...
        '%+.4f^{\\circ}/g,\\;R^2 = %.3f$'], ...
        direction_condition,model.offset,model.slope,model.r_squared);
    plot(ax,acceleration_fit,polyval(model.coefficients,acceleration_fit),'-', ...
        'Color',fit_color,'LineWidth',2,'DisplayName',fit_label);
end

xline(ax,0,'--k','HandleVisibility','off');
yline(ax,0,'--k','HandleVisibility','off');
ylabel(ax,'bearing error [deg]');
title(ax,direction_title);
legend(ax,'show','Location','northwest','Interpreter','latex');
end

function model = fitLinearSensitivity(input_values,output_values)
input_values = double(input_values(:));
output_values = double(output_values(:));
finite_values = isfinite(input_values) & isfinite(output_values);
input_values = input_values(finite_values);
output_values = output_values(finite_values);

model.valid = false;
model.samples = numel(input_values);
model.coefficients = [NaN NaN];
model.slope = NaN;
model.offset = NaN;
model.r_squared = NaN;

if model.samples < 2 || ...
        max(input_values) - min(input_values) <= sqrt(eps)
    return;
end

model.coefficients = polyfit(input_values,output_values,1);
prediction = polyval(model.coefficients,input_values);
sum_squared_residuals = sum((output_values - prediction).^2);
sum_squared_total = sum((output_values - mean(output_values)).^2);
if sum_squared_total > 0
    model.r_squared = 1 - sum_squared_residuals/sum_squared_total;
end

model.slope = model.coefficients(1);
model.offset = model.coefficients(2);
model.valid = true;
end

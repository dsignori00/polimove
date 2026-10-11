%% CALIBRATION_ERRORS
% Per sensor in sensors{}:
%   lateral:      ey = TY + tan(dpsi) * x_rel
%   longitudinal: ex = TX (constant mean)

if ~exist('err_thr','var'), err_thr = 10; end
MAX_GAP_S = 0.2;   % max distance from nearest GT sample [s]

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

    ex  = x - interp1(t_gt,x_gt,tq,'linear',NaN);
    ey  = double(S.y_rel(:)) - interp1(t_gt,y_gt,tq,'linear',NaN);
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

    p    = polyfit(x(ok),ey(ok),1);
    dpsi = atand(p(1));
    TX   = mean(ex(ok));
    ey_fit = polyval(p,x(ok));
    ss_res = sum((ey(ok) - ey_fit).^2);
    ss_tot = sum((ey(ok) - mean(ey(ok))).^2);
    if ss_tot > 0
        R2 = 1 - ss_res/ss_tot;
    else
        R2 = NaN;
    end
    fprintf(['%-8s  N = %5d | TX = %+.3f m | TY = %+.3f m | ' ...
        'dpsi = %+.3f deg | R2 = %.3f\n'], ...
        sensors{i}.name,nnz(ok),TX,p(2),dpsi,R2);

    sensor_tab = uitab(calibration_tabs,'Title',sensors{i}.name);
    sensor_layout = tiledlayout(sensor_tab,2,1, ...
        'Padding','compact','TileSpacing','compact');

    % Lateral error vs x
    nexttile(sensor_layout); hold on; grid on; box on
    plot(x(ok),ey(ok),'.', ...
        'Color',sensors{i}.col, ...
        'MarkerSize',6, ...
        'DisplayName','Measurement error');
    xf = linspace(min(x(ok)),max(x(ok)),2);
    fit_label = sprintf(['Fit: $T_Y = %.3f\\,\\mathrm{m},\\;' ...
        '\\Delta\\psi = %.2f^{\\circ},\\;R^2 = %.3f$'],p(2),dpsi,R2);
    plot(xf,polyval(p,xf),'k-', ...
        'LineWidth',2, ...
        'DisplayName',fit_label);
    yline(0,'--k','HandleVisibility','off');
    ylim([-1 1]); ylabel('y error [m]'); title([sensors{i}.name ' - lateral']);
    legend('show','Location','northwest');
    xlabel('x relative [m]');

    % Longitudinal error vs time
    nexttile(sensor_layout); hold on; grid on; box on
    t = tq - t_gt(1);
    plot(t(ok),ex(ok),'.', ...
        'Color',sensors{i}.col, ...
        'MarkerSize',6, ...
        'DisplayName','Measurement error');
    mean_label = sprintf('Mean: $T_X = %.3f\\,\\mathrm{m}$',TX);
    yline(TX,'k-', ...
        'LineWidth',2, ...
        'DisplayName',mean_label);
    yline(0,'--k','HandleVisibility','off');
    ylim([-1 1]); ylabel('x error [m]'); title([sensors{i}.name ' - longitudinal']);
    legend('show','Location','northwest');
    xlabel('time [s]');
end

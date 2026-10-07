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

    p    = polyfit(x(ok),ey(ok),1);
    dpsi = atand(p(1));
    TX   = mean(ex(ok));
    fprintf('%-8s  N = %5d | TX = %+.3f m | TY = %+.3f m | dpsi = %+.3f deg\n', ...
        sensors{i}.name, nnz(ok), TX, p(2), dpsi);

    figure('Name',['Extrinsic errors - ' sensors{i}.name],'Color','w');
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');

    % Lateral error vs x
    nexttile; hold on; grid on; box on
    plot(x(ok),ey(ok),'.','Color',sensors{i}.col,'MarkerSize',6);
    xf = linspace(min(x(ok)),max(x(ok)),2);
    plot(xf,polyval(p,xf),'k-','LineWidth',2);
    yline(0,'--k');
    ylim([-1 1]); ylabel('y error [m]'); title([sensors{i}.name ' - lateral']);
    legend('Measurement error', ...
        sprintf('Fit: T_Y = %.3f m, \\Delta\\psi = %.2f deg',p(2),dpsi), ...
        'Location','northwest');
    xlabel('x relative [m]');

    % Longitudinal error vs time
    nexttile; hold on; grid on; box on
    t = tq - t_gt(1);
    plot(t(ok),ex(ok),'.','Color',sensors{i}.col,'MarkerSize',6);
    yline(TX,'k-','LineWidth',2);
    yline(0,'--k');
    ylim([-1 1]); ylabel('x error [m]'); title([sensors{i}.name ' - longitudinal']);
    legend('Measurement error',sprintf('Mean: T_X = %.3f m',TX), ...
        'Location','northwest');
    xlabel('time [s]');
end
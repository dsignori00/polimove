%% EXTRINSIC_ERRORS
% Extrinsic calibration error analysis for:
%   - LiDAR Clustering
%   - LiDAR PointPillars
%
% Yaw misalignment (dpsi) estimated from the lateral error model:
%   ey = TY + tan(dpsi) * x_rel
% (TY = lateral offset, byproduct of the fit)
%
% Figure: lateral error vs longitudinal distance
%   2) longitudinal error vs time

%% CONFIG
K_SENS_LIST = [1 2];

DISP_NAMES = { ...
    'Lidar Clustering', ...
    'Lidar PointPillars'};

T_FIELD   = 'sens_stamp';   % GT interpolation time: 'sens_stamp' | 'stamp'
MAX_GAP_S = 0.2;            % max distance from nearest GT sample [s]
DEDUP_TOL = 1e-3;           % duplicate tolerance [s]

if ~exist('err_thr','var') || isempty(err_thr), err_thr = 1.5; end
if ~exist('f','var')       || isempty(f),       f = 1; end

if ~exist('sensors','var')
    error('extrinsic_errors: sensors{} not found.');
end

if ~exist('gt','var')
    error('extrinsic_errors: gt not found.');
end


%% ============================================================
% PREPARE GROUND TRUTH
% ============================================================

t_gt = double(gt.stamp(:));
x_gt = double(gt.x_rel(:));
y_gt = double(gt.y_rel(:));

valid_gt = ...
    isfinite(t_gt) & ...
    isfinite(x_gt) & ...
    isfinite(y_gt);

t_gt = t_gt(valid_gt);
x_gt = x_gt(valid_gt);
y_gt = y_gt(valid_gt);

[t_gt,iu] = unique(t_gt);

x_gt = x_gt(iu);
y_gt = y_gt(iu);


%% ============================================================
% PROCESS SENSORS
% ============================================================

RES = cell(size(sensors));

for i = K_SENS_LIST

    S = sensors{i}.s;

    name = DISP_NAMES{i};

    % --- detection data
    t     = double(S.stamp(:));
    tq    = double(S.(T_FIELD)(:));

    x_det = double(S.x_rel(:));
    y_det = double(S.y_rel(:));


    %% GT interpolated at detection time

    x_gi = interp1(t_gt,x_gt,tq,'linear',NaN);
    y_gi = interp1(t_gt,y_gt,tq,'linear',NaN);


    %% Distance from nearest GT sample

    i_nn = interp1( ...
        t_gt, ...
        (1:numel(t_gt))', ...
        tq, ...
        'nearest', ...
        NaN);

    gap = nan(size(tq));

    v = isfinite(i_nn);

    gap(v) = ...
        abs(t_gt(i_nn(v)) - tq(v));


    %% Relative-frame errors

    ex = x_det - x_gi;
    ey = y_det - y_gi;


    %% Valid samples

    ok = ...
        isfinite(t) & ...
        isfinite(tq) & ...
        isfinite(ex) & ...
        isfinite(ey) & ...
        gap <= MAX_GAP_S & ...
        abs(ex) < err_thr & ...
        abs(ey) < err_thr;


    %% Remove duplicated scans

    if ~isempty(DEDUP_TOL)

        [~,idx] = sort(tq);

        dup = false(size(tq));

        if numel(idx) > 1
            dup(idx(2:end)) = ...
                diff(tq(idx)) < DEDUP_TOL;
        end

        ok = ok & ~dup;

    end


    if nnz(ok) < 3

        warning( ...
            '%s: too few valid samples (%d). Sensor skipped.', ...
            name,nnz(ok));

        continue
    end


    %% ========================================================
    % EXTRINSIC ESTIMATION
    % =========================================================

    % Lateral error model:
    %
    % ey = TY + tan(dpsi) * x_rel

    p = polyfit( ...
        x_det(ok), ...
        ey(ok), ...
        1);

    TY   = p(2);
    dpsi = atand(p(1));


    %% Console output

    fprintf('\n=== %s ===\n',name);

    fprintf( ...
        'Valid samples: %d / %d | gate %.1f m | max GT gap %.2f s\n', ...
        nnz(ok),numel(t),err_thr,MAX_GAP_S);

    fprintf( ...
        'dpsi = %+.3f deg | TY = %+.3f m\n', ...
        dpsi,TY);


    %% Store results

    RES{i}.x_det = x_det;
    RES{i}.ey    = ey;

    RES{i}.ok = ok;

    RES{i}.p     = p;
    RES{i}.TY    = TY;
    RES{i}.dpsi  = dpsi;

end


%% ============================================================
% FIGURE
% LATERAL ERROR VS LONGITUDINAL DISTANCE
% ============================================================

figure( ...
    'Name','Extrinsic error - lateral', ...
    'Color','w');

tiledlayout( ...
    numel(K_SENS_LIST),1, ...
    'Padding','compact', ...
    'TileSpacing','compact');


for i = K_SENS_LIST

    if isempty(RES{i})
        continue
    end

    R = RES{i};

    ax(f) = nexttile;
    hold on
    grid on
    box on
    f = f+1;

    % Sensor data
    plot( ...
        R.x_det(R.ok), ...
        R.ey(R.ok), ...
        '.', ...
        'Color',sensors{i}.col, ...
        'MarkerSize',6, ...
        'DisplayName','Measurement error');


    % Linear fit
    xf = linspace( ...
        min(R.x_det(R.ok)), ...
        max(R.x_det(R.ok)), ...
        200);

    plot( ...
        xf, ...
        polyval(R.p,xf), ...
        'k-', ...
        'LineWidth',2, ...
        'DisplayName','Linear fit');


    yline( ...
        0, ...
        '--k', ...
        'LineWidth',0.3, ...
        'HandleVisibility','off');


    ylabel('y error [m]');

    ylim([-1 1]);

    title(DISP_NAMES{i});


    legend( ...
        'Measurement error', ...
        sprintf( ...
        'Fit: T_Y = %.3f m, \\Delta\\psi = %.2f deg', ...
        R.TY,R.dpsi), ...
        'Location','northwest', ...
        'Interpreter','tex');

end

xlabel('x relative [m]');
%% R_ADAPTIVE_PLOT
% Measurement error compared with the final adaptive covariance:
%
%   r_final_xx
%   r_final_yy
%
% Sensors:
%   - LiDAR Clustering
%   - LiDAR PointPillars
%
% If R is statistically consistent:
%   ~68% of samples should lie within +/-1 sigma
%   ~95% of samples should lie within +/-2 sigma
%
% Optional recentering estimates a local residual bias and centers the
% covariance bands around it.

%% CONFIG
K_SENS_LIST = [1 2];

DISP_NAMES = { ...
    'Lidar Clustering', ...
    'Lidar PointPillars'};

OPP_IDX   = 1;

R_FRAME   = 'map';        % 'map' | 'rel'
R_ALIGN   = 'next';       % 'next' | 'previous' | 'nearest'

COMP_MODE = 'estimate';   % 'estimate' | 'workspace' | 'none'
BETA      = 0.0058;       % [rad]

DEDUP_TOL = 1e-3;         % [s]

SHOW_RECENTER = true;
BIAS_WIN_S    = 1.0;      % [s]
BIAS_MIN_N    = 3;
GAP_S         = 0.5;      % [s]

if ~exist('err_thr','var') || isempty(err_thr), err_thr = 5; end
if ~exist('f','var')       || isempty(f),       f = 1; end

if ~exist('log','var')
    error('R_adaptive_plot: log not found.');
end

if ~exist('sensors','var')
    error('R_adaptive_plot: sensors{} not found.');
end

c_sig  = [0 0 0.55];
c_2sig = [0 0.45 1];
c_bias = [0.85 0.10 0.10];

RES = cell(size(sensors));


%% ============================================================
% PROCESS SENSORS
% ============================================================

for i = K_SENS_LIST

    S = sensors{i}.s;

    name = DISP_NAMES{i};
    id   = sensors{i}.id;
    col  = sensors{i}.col;

    %% Measurement data

    t       = double(S.stamp(:));
    ts      = double(S.sens_stamp(:));

    ex      = double(S.x_map_err(:));
    ey      = double(S.y_map_err(:));

    xr      = double(S.x_rel(:));
    yr      = double(S.y_rel(:));

    rho     = hypot(xr,yr);

    psi_tgt = double(S.yaw_map(:));
    psi_ego = psi_tgt - double(S.yaw_rel(:));

    los = psi_ego + atan2d(yr,xr);


    %% Valid measurements

    ok = ...
        isfinite(t) & ...
        isfinite(ts) & ...
        isfinite(ex) & ...
        isfinite(ey) & ...
        isfinite(rho) & ...
        isfinite(psi_tgt) & ...
        isfinite(psi_ego) & ...
        hypot(ex,ey) < err_thr;


    %% Remove repeated scans

    if ~isempty(DEDUP_TOL)

        [~,idx] = sort(ts);

        dup = false(size(ts));

        if numel(idx) > 1
            dup(idx(2:end)) = diff(ts(idx)) < DEDUP_TOL;
        end

        ok = ok & ~dup;

    end


    %% ========================================================
    % BIAS COMPENSATION
    % =========================================================

    mode = lower(COMP_MODE);

    if strcmp(mode,'workspace') && ...
       ~(exist('PAR','var') && isstruct(PAR) && ...
         all(isfield(PAR,{'BETA','D_LONG','C_LAT'})))

        mode = 'estimate';

    end


    switch mode

        case 'workspace'

            P = PAR;


        case 'estimate'

            P.BETA = BETA;

            ct = cosd(psi_tgt);
            st = sind(psi_tgt);

            cl = cosd(los);
            sl = sind(los);

            A = [ ...
                ct(ok), -sl(ok);
                st(ok),  cl(ok)];

            y = [ ...
                ex(ok) + P.BETA.*sl(ok).*rho(ok);
                ey(ok) - P.BETA.*cl(ok).*rho(ok)];

            p = A \ y;

            P.D_LONG = p(1);
            P.C_LAT  = p(2);


        case 'none'

            P = struct( ...
                'BETA',0, ...
                'D_LONG',0, ...
                'C_LAT',0);


        otherwise

            error('Unknown COMP_MODE: %s',COMP_MODE);

    end


    [dx,dy] = bias_model(psi_tgt,los,rho,P);

    ex_c = ex - dx;
    ey_c = ey - dy;


    %% ========================================================
    % FINAL R
    % =========================================================

    fld_x = sprintf('opponents__%s_r_final_xx',id);
    fld_y = sprintf('opponents__%s_r_final_yy',id);

    TT = local_find_tt(log,fld_x,fld_y);

    if isempty(TT)

        warning( ...
            '%s: %s / %s not found. Sensor skipped.', ...
            name,fld_x,fld_y);

        continue
    end


    tr = double(TT.stamp__tot(:));

    Rxx = double(TT.(fld_x)(:,min(OPP_IDX,size(TT.(fld_x),2))));
    Ryy = double(TT.(fld_y)(:,min(OPP_IDX,size(TT.(fld_y),2))));

    Rxx(Rxx <= -9999 | Rxx <= 0) = NaN;
    Ryy(Ryy <= -9999 | Ryy <= 0) = NaN;

    sx = local_align(tr,sqrt(Rxx),t,R_ALIGN);
    sy = local_align(tr,sqrt(Ryy),t,R_ALIGN);


    %% ========================================================
    % COMPARISON FRAME
    % =========================================================

    if strcmpi(R_FRAME,'rel')

        [cu,cv] = map2frame(ex_c,ey_c,psi_ego);

        label_x = 'x relative error [m]';
        label_y = 'y relative error [m]';

    else

        cu = ex_c;
        cv = ey_c;

        label_x = 'x map error [m]';
        label_y = 'y map error [m]';

    end


    ok_x = ok & isfinite(cu) & isfinite(sx) & sx > 1e-4;
    ok_y = ok & isfinite(cv) & isfinite(sy) & sy > 1e-4;


    %% Coverage

    coverage = @(e,s,m,k) ...
        100*mean(abs(e(m)) <= k*s(m));


    %% ========================================================
    % LOCAL RECENTERING
    % =========================================================

    bx = nan(size(cu));
    by = nan(size(cv));

    rx = nan(size(cu));
    ry = nan(size(cv));

    ok_rx = false(size(ok));
    ok_ry = false(size(ok));


    if SHOW_RECENTER

        w = BIAS_WIN_S/2;

        bx(ok_x) = local_win_mean( ...
            ts(ok_x),cu(ok_x),w,w,BIAS_MIN_N);

        by(ok_y) = local_win_mean( ...
            ts(ok_y),cv(ok_y),w,w,BIAS_MIN_N);

        rx = cu - bx;
        ry = cv - by;

        ok_rx = ok_x & isfinite(bx);
        ok_ry = ok_y & isfinite(by);

    end


    %% ========================================================
    % STORE
    % =========================================================

    RES{i}.name = name;
    RES{i}.col  = col;

    RES{i}.t  = t;

    RES{i}.x  = cu;
    RES{i}.y  = cv;

    RES{i}.sx = sx;
    RES{i}.sy = sy;

    RES{i}.ok_x = ok_x;
    RES{i}.ok_y = ok_y;

    RES{i}.label_x = label_x;
    RES{i}.label_y = label_y;

    RES{i}.bx = bx;
    RES{i}.by = by;

    RES{i}.rx = rx;
    RES{i}.ry = ry;

    RES{i}.ok_rx = ok_rx;
    RES{i}.ok_ry = ok_ry;

    RES{i}.cov_x1 = coverage(cu,sx,ok_x,1);
    RES{i}.cov_x2 = coverage(cu,sx,ok_x,2);

    RES{i}.cov_y1 = coverage(cv,sy,ok_y,1);
    RES{i}.cov_y2 = coverage(cv,sy,ok_y,2);


    if SHOW_RECENTER

        RES{i}.cov_rx1 = coverage(rx,sx,ok_rx,1);
        RES{i}.cov_rx2 = coverage(rx,sx,ok_rx,2);

        RES{i}.cov_ry1 = coverage(ry,sy,ok_ry,1);
        RES{i}.cov_ry2 = coverage(ry,sy,ok_ry,2);

    end


    %% Console

    fprintf('\n=== %s ===\n',name);

    fprintf( ...
        'Bias: D_LONG %+.3f m | C_LAT %+.3f m | BETA %+.3f deg\n', ...
        P.D_LONG,P.C_LAT,rad2deg(P.BETA));

    fprintf( ...
        'X coverage: 1 sigma %.1f%% | 2 sigma %.1f%%\n', ...
        RES{i}.cov_x1,RES{i}.cov_x2);

    fprintf( ...
        'Y coverage: 1 sigma %.1f%% | 2 sigma %.1f%%\n', ...
        RES{i}.cov_y1,RES{i}.cov_y2);

end


%% ============================================================
% FIGURE 1 - X ERROR
% ============================================================

figure('Name','R adaptive - X','Color','w')

tiledlayout( ...
    numel(K_SENS_LIST),1, ...
    'Padding','compact', ...
    'TileSpacing','compact');


for i = K_SENS_LIST

    if isempty(RES{i}), continue; end

    R = RES{i};

    ax(f) = nexttile;
    hold on; grid on; box on
    f = f+1;

    plot_R_panel( ...
        R.t,R.x,R.sx,R.ok_x, ...
        R.col,c_sig,c_2sig);

    ylabel(R.label_x)

    title(sprintf( ...
        '%s   |   1\\sigma %.0f%%   2\\sigma %.0f%%   (68%% / 95%%)', ...
        R.name,R.cov_x1,R.cov_x2), ...
        'Interpreter','tex');

end

xlabel('timestamp [s]');


%% ============================================================
% FIGURE 2 - Y ERROR
% ============================================================

figure('Name','R adaptive - Y','Color','w')

tiledlayout( ...
    numel(K_SENS_LIST),1, ...
    'Padding','compact', ...
    'TileSpacing','compact');


for i = K_SENS_LIST

    if isempty(RES{i}), continue; end

    R = RES{i};

    ax(f) = nexttile;
    hold on; grid on; box on
    f = f+1;

    plot_R_panel( ...
        R.t,R.y,R.sy,R.ok_y, ...
        R.col,c_sig,c_2sig);

    ylabel(R.label_y)

    title(sprintf( ...
        '%s   |   1\\sigma %.0f%%   2\\sigma %.0f%%   (68%% / 95%%)', ...
        R.name,R.cov_y1,R.cov_y2), ...
        'Interpreter','tex');

end

xlabel('timestamp [s]');


%% ============================================================
% RECENTERED FIGURES
% ============================================================

if SHOW_RECENTER

    %% X

    figure('Name','R adaptive recentered - X','Color','w')

    tiledlayout( ...
        numel(K_SENS_LIST),1, ...
        'Padding','compact', ...
        'TileSpacing','compact');


    for i = K_SENS_LIST

        if isempty(RES{i}), continue; end

        R = RES{i};

        ax(f) = nexttile;
        hold on; grid on; box on
        f = f+1;

        plot_R_recentered( ...
            R.t,R.x,R.bx,R.sx,R.ok_rx, ...
            R.col,c_bias,c_sig,c_2sig,GAP_S);

        ylabel(R.label_x)

        title(sprintf( ...
            '%s   |   1\\sigma %.0f%% \\rightarrow %.0f%%   2\\sigma %.0f%% \\rightarrow %.0f%%', ...
            R.name,R.cov_x1,R.cov_rx1,R.cov_x2,R.cov_rx2), ...
            'Interpreter','tex');

    end

    xlabel('timestamp [s]');


    %% Y

    figure('Name','R adaptive recentered - Y','Color','w')

    tiledlayout( ...
        numel(K_SENS_LIST),1, ...
        'Padding','compact', ...
        'TileSpacing','compact');


    for i = K_SENS_LIST

        if isempty(RES{i}), continue; end

        R = RES{i};

        ax(f) = nexttile;
        hold on; grid on; box on
        f = f+1;

        plot_R_recentered( ...
            R.t,R.y,R.by,R.sy,R.ok_ry, ...
            R.col,c_bias,c_sig,c_2sig,GAP_S);

        ylabel(R.label_y)

        title(sprintf( ...
            '%s   |   1\\sigma %.0f%% \\rightarrow %.0f%%   2\\sigma %.0f%% \\rightarrow %.0f%%', ...
            R.name,R.cov_y1,R.cov_ry1,R.cov_y2,R.cov_ry2), ...
            'Interpreter','tex');

    end

    xlabel('timestamp [s]');

end


%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function plot_R_panel(t,e,s,ok,col,c1,c2)

[ts,idx] = sort(t);
ss = s(idx);

plot(t(ok),e(ok),'.', ...
    'Color',col,'MarkerSize',9, ...
    'DisplayName','Compensated error');

plot(ts, ss,'-','Color',c1,'LineWidth',1.3, ...
    'DisplayName','\pm\sigma');

plot(ts,-ss,'-','Color',c1,'LineWidth',1.3, ...
    'HandleVisibility','off');

plot(ts, 2*ss,'-','Color',c2,'LineWidth',1.0, ...
    'DisplayName','\pm2\sigma');

plot(ts,-2*ss,'-','Color',c2,'LineWidth',1.0, ...
    'HandleVisibility','off');

yline(0,'--k','HandleVisibility','off');

legend('Location','northeast','NumColumns',2,'Interpreter','tex');
end


function plot_R_recentered(t,e,b,s,ok,col,cb,c1,c2,gap)

[ts,idx] = sort(t(ok));

es = e(ok); es = es(idx);
bs = b(ok); bs = bs(idx);
ss = s(ok); ss = ss(idx);

plot(ts,es,'.', ...
    'Color',col,'MarkerSize',9, ...
    'DisplayName','Compensated error');

[tg,Y] = local_gapnan( ...
    ts,[bs,bs+ss,bs-ss,bs+2*ss,bs-2*ss],gap);

plot(tg,Y(:,1),'-','Color',cb,'LineWidth',1.6, ...
    'DisplayName','Local mean');

plot(tg,Y(:,2),'-','Color',c1,'LineWidth',1.3, ...
    'DisplayName','Mean \pm\sigma');

plot(tg,Y(:,3),'-','Color',c1,'LineWidth',1.3, ...
    'HandleVisibility','off');

plot(tg,Y(:,4),'-','Color',c2,'LineWidth',1.0, ...
    'DisplayName','Mean \pm2\sigma');

plot(tg,Y(:,5),'-','Color',c2,'LineWidth',1.0, ...
    'HandleVisibility','off');

yline(0,'--k','HandleVisibility','off');

legend('Location','northeast','NumColumns',2,'Interpreter','tex');
end


function [dx,dy] = bias_model(psi_tgt,los,rho,P)

ct = cosd(psi_tgt);
st = sind(psi_tgt);

cl = cosd(los);
sl = sind(los);

lat = rho.*P.BETA + P.C_LAT;

dx = ct.*P.D_LONG - sl.*lat;
dy = st.*P.D_LONG + cl.*lat;

dx(~isfinite(dx)) = 0;
dy(~isfinite(dy)) = 0;
end


function [a,b] = map2frame(ex,ey,psi)

c = cosd(psi(:));
s = sind(psi(:));

a =  c.*ex(:) + s.*ey(:);
b = -s.*ex(:) + c.*ey(:);
end


function m = local_win_mean(t,x,w_back,w_fwd,min_n)

t = t(:);
x = x(:);

[ts,is] = sort(t);
xs = x(is);

n = numel(ts);

m_sorted = nan(n,1);

for i = 1:n

    idx = ...
        ts >= ts(i)-w_back & ...
        ts <= ts(i)+w_fwd;

    idx(i) = false;

    if nnz(idx) >= min_n
        m_sorted(i) = mean(xs(idx),'omitnan');
    end
end

m = nan(n,1);
m(is) = m_sorted;
end


function [tg,Y] = local_gapnan(t,Y,gap_s)

t = t(:);

breaks = find(diff(t) > gap_s);

if isempty(breaks)
    tg = t;
    return
end

n  = numel(t);
nb = numel(breaks);

tg = nan(n+nb,1);
Yg = nan(n+nb,size(Y,2));

src = 1;
dst = 1;

for k = 1:nb

    len = breaks(k)-src+1;

    tg(dst:dst+len-1)   = t(src:breaks(k));
    Yg(dst:dst+len-1,:) = Y(src:breaks(k),:);

    dst = dst+len+1;
    src = breaks(k)+1;
end

len = n-src+1;

tg(dst:dst+len-1)   = t(src:n);
Yg(dst:dst+len-1,:) = Y(src:n,:);

Y = Yg;
end


function TT = local_find_tt(log,varargin)

TT = [];

if ~isstruct(log) || ~isscalar(log)
    return
end

if local_has(log,varargin{:})
    TT = log;
    return
end

fn = fieldnames(log);

for i = 1:numel(fn)

    v = log.(fn{i});

    if isstruct(v) && isscalar(v) && local_has(v,varargin{:})
        TT = v;
        return
    end
end
end


function tf = local_has(s,varargin)

tf = isfield(s,'stamp__tot');

for i = 1:numel(varargin)
    tf = tf && isfield(s,varargin{i});
end
end


function vq = local_align(t_src,v_src,t_q,method)

t_src = t_src(:);
v_src = v_src(:);
t_q   = t_q(:);

vq = nan(size(t_q));

good = isfinite(t_src) & isfinite(v_src);

if nnz(good) < 2
    return
end

t_src = t_src(good);
v_src = v_src(good);

[t_src,iu] = unique(t_src,'stable');
v_src = v_src(iu);

[t_src,is] = sort(t_src);
v_src = v_src(is);

ok = isfinite(t_q);

vq(ok) = interp1( ...
    t_src,v_src,t_q(ok),method,NaN);
end
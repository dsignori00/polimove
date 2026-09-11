%% ====================================================================
%  Lag/latency analysis of opponent tracking - v_x only
%
%  Break point (maneuver onset), based on velocity:
%    tbreak = min{ t : dir*(v(t) - v0) > frac*|v0|  for at least holdT }
%    dOnset = tbreak,estimated - tbreak,GT     (+ = DELAY, - = lead)
%
%  GT and estimate are the SAME quantity, on the SAME uniform grid, with
%  the SAME v0 plateau and the SAME relative threshold: the comparison is
%  symmetric by construction, with no bandwidth or noise compensation.
%
%  Acceleration is used only as the smoothed derivative of v_x to segment
%  maneuvers and separate steady-state/transient samples for the RMSE.
%
%  NOTE on the buffer-age axis (dLag): the 0 ms reference point is
%  anchored to lag 1 (not lag 0). dLag is rebased right after its
%  computation by subtracting dLag(2) (= lag 1, since col = N-(0:nLag-1)
%  and the printed label is j-1) from every entry. Lag 0, being fresher
%  than lag 1, therefore reads a small NEGATIVE buffer age after the
%  rebase. This only affects the displayed/printed buffer-age values
%  (table, Figure 3, Figure 4); it does not change tnow/lr, the masks,
%  or the onset-delay (dOnset) computation, which are unaffected.
% ====================================================================

%% ==================== PARAMETERS ====================
sl        = 1;                    % opponent slot
fld       = 'opponents__vx';      % velocity field in the buffer
nLag      = 15;                   % number of lags to analyze
gapMax    = 0.30;                 % s: gap above which plotted lines are split
useStamp  = false;                % true = lag 0 at the publication timestamp
dtU       = 0.02;                 % s: uniform working-grid step

% --- v_x derivative (maneuver segmentation and regime classification) ---
wSmooth   = 0.20;                 % s: smoothing for the derivative used in the RMSE
wSmoothEv = 0.50;                 % s: smoothing for maneuver segmentation
aThr      = 2.0;                  % m/s^2: steady-state/transient threshold for the RMSE
aOn       = 1.0;                  % m/s^2: event activation threshold
aOff      = 0.5;                  % m/s^2: event deactivation threshold (hysteresis, aOff < aOn)
minGapEv  = 0.30;                 % s: closer events are merged

% --- onset based on v_x ---
brkFrac   = [0.02 0.05];          % break thresholds, fraction of the |v0| plateau
holdT     = 0.08;                 % s: required persistence (60-100 ms)
preWin    = 1.00;                 % s: margin before the event for onset detection
minDv     = 1.0;                  % m/s: minimum v_x change required to validate a maneuver
useCommon = true;                 % true = statistics only on events valid for ALL lags

%% ==================== BUFFER EXTRACTION ====================
oh = log.perception__opponents_history;
T  = double(oh.('timestamp[]__tot'));
nv = double(oh.opponents__steps(:, sl));

st = double(log.perception__opponents.stamp__tot);  st(st==0) = NaN;
t0 = min(st, [], 'omitnan');

V = maskByStamps(squeeze(double(oh.(fld)(:, sl, :))), nv);
T = maskByStamps(T, nv);

V = fliplr(V);  T = fliplr(T);            % end column = "now"
if useStamp && numel(st) == size(T,1), T(:,end) = st; end
T = T - t0;

tnow = nan(size(T,1),1);
for r = 1:size(T,1)
    li = find(isfinite(T(r,:)), 1, 'last');
    if ~isempty(li), tnow(r) = T(r,li); end
end

%% ==================== LAG MASKS ====================
N    = size(V,2);
nLag = min(nLag, N);
col  = N - (0:nLag-1);
M    = false(size(V,1), nLag);
L    = nan(size(V,1), nLag);

for j = 1:nLag
    c  = col(j);
    lr = tnow - T(:,c);
    ok = nv >= j & isfinite(T(:,c)) & isfinite(V(:,c)) & V(:,c) ~= 0 ...
         & isfinite(lr) & lr >= 0;
    if nnz(ok) > 5
        lm = median(lr(ok));
        ok = ok & abs(lr - lm) < max(0.05, 0.5*lm);
    end
    M(:,j) = ok;  L(:,j) = lr;
end

dLag = nan(nLag,1);
for j = 1:nLag
    if any(M(:,j)), dLag(j) = median(L(M(:,j), j)) * 1e3; end
end

% --- rebase buffer-age axis: 0 ms reference = lag 1 (not lag 0) ---
if nLag >= 2 && isfinite(dLag(2))
    dLag = dLag - dLag(2);
end

SS = buildLagSeries(V, T, M, col, tnow);   % estimated v_x series, by lag

%% ==================== GROUND TRUTH ====================
gtT = double(gt.stamp) - t0;  gtV = double(gt.vx);
g   = isfinite(gtT) & isfinite(gtV);
[gtTu, iu] = unique(gtT(g));  gtVu = gtV(g);  gtVu = gtVu(iu);

dtGt   = median(diff(gtTu));
aGtCls = gradient(movmean(gtVu, max(3, round(wSmooth  /dtGt))), gtTu);  % regime / RMSE
aGtEv  = gradient(movmean(gtVu, max(3, round(wSmoothEv/dtGt))), gtTu);  % segmentation

%% ==================== MANEUVER EVENTS ====================
ev    = detectEvents(gtTu, aGtEv, aOn, aOff, minGapEv);
dirEv = nan(size(ev,1),1);
v0Ev  = nan(size(ev,1),1);

if ~isempty(ev)
    keepEv = true(size(ev,1),1);
    for m = 1:size(ev,1)
        idx = gtTu >= ev(m,1) & gtTu <= ev(m,2);
        if nnz(idx) < 3 || (max(gtVu(idx)) - min(gtVu(idx))) < minDv
            keepEv(m) = false;  continue;
        end
        pre  = gtTu >= ev(m,1) - preWin & gtTu < ev(m,1);
        vEnd = median(gtVu(gtTu > ev(m,2) - 0.2 & gtTu <= ev(m,2)), 'omitnan');
        v0Ev(m)  = median(gtVu(pre), 'omitnan');
        dirEv(m) = sign(vEnd - v0Ev(m));
        if ~isfinite(dirEv(m)) || dirEv(m) == 0 || ~isfinite(v0Ev(m))
            keepEv(m) = false;
        end
    end
    fprintf(' events detected: %d, discarded (Delta v < %.1f m/s or zero direction): %d\n', ...
            numel(keepEv), minDv, nnz(~keepEv));
    ev = ev(keepEv,:);  dirEv = dirEv(keepEv);  v0Ev = v0Ev(keepEv);
end

%% ==================== ONSET BASED ON v_x ====================
nH = max(2, round(holdT/dtU));
nF = numel(brkFrac);

tBrkGt  = nan(size(ev,1), nF);
tBrkEst = nan(size(ev,1), nLag, nF);

for m = 1:size(ev,1)
    tu = (ev(m,1) - preWin : dtU : ev(m,2))';
    yg = interp1(gtTu, gtVu, tu, 'linear', NaN);
    if nnz(isfinite(yg)) < 10, continue; end

    for f = 1:nF
        tBrkGt(m,f) = breakVx(tu, yg, v0Ev(m), dirEv(m), brkFrac(f), nH);
    end
    if all(isnan(tBrkGt(m,:))), continue; end

    for j = 1:nLag
        if isempty(SS{j}), continue; end
        tt = SS{j}(:,1);
        if tt(1) > tu(1) || tt(end) < tu(end), continue; end
        ye = interp1(tt, SS{j}(:,2), tu, 'linear', NaN);
        if nnz(isfinite(ye)) < 0.8*numel(tu), continue; end
        for f = 1:nF
            tBrkEst(m,j,f) = breakVx(tu, ye, v0Ev(m), dirEv(m), brkFrac(f), nH);
        end
    end
end

dOnset = (tBrkEst - reshape(tBrkGt, size(tBrkGt,1), 1, nF)) * 1e3;   % ms; + = delay, - = lead

%% ==================== COMMON EVENT SET ====================
% Without this, each lag would have a different maneuver population and
% the medians would not be comparable.
okEv = squeeze(all(isfinite(dOnset), 2));   % [nEv x nF]
if useCommon
    for f = 1:nF
        bad = ~okEv(:,f);
        dOnset(bad,:,f) = NaN;
    end
end

Dmed = nan(nLag, nF);
nOn = zeros(nLag, nF);
for f = 1:nF
    [Dmed(:,f), nOn(:,f)] = lagStats(dOnset(:,:,f), nLag);
end

%% ==================== RMSE vs GT(t-d), BY REGIME ====================
common = all(M, 2) & isfinite(tnow);
for j = 1:nLag
    c = col(j);
    common = common & T(:,c) >= gtTu(1) & T(:,c) <= gtTu(end);
end

R  = nan(nLag, 4);   % [delay, RMSE_all, RMSE_steady, RMSE_tran]
nT = nan(nLag, 1);
for j = 1:nLag
    c = col(j);
    if nnz(common) < 10, continue; end
    tt = T(common,c);
    e  = V(common,c) - interp1(gtTu, gtVu, tt, 'linear', NaN);
    tr = abs(interp1(gtTu, aGtCls, tt, 'linear', NaN)) > aThr;
    nT(j) = nnz(tr);
    R(j,:) = [dLag(j), ...
              sqrt(mean(e.^2,     'omitnan')), ...
              sqrt(mean(e(~tr).^2,'omitnan')), ...
              sqrt(mean(e( tr).^2,'omitnan'))];
end

%% ==================== OUTPUT ====================
fprintf('\n onset based on v_x   plateau thresholds %s, hold %.0f ms\n', ...
        mat2str(brkFrac*100), holdT*1e3);
fprintf(' common samples = %d   (transient = %d, %.0f%%)\n', ...
        nnz(common), nT(1), 100*nT(1)/max(1,nnz(common)));
fprintf(' valid maneuvers = %d   (events common to all lags: %s)\n\n', ...
        size(ev,1), mat2str(sum(okEv,1)));

fprintf(' lag  delay[ms]   RMSE_all  RMSE_steady  RMSE_tran\n');
fprintf('      (0 ms = lag 1)\n');
for j = 1:nLag
    if isfinite(R(j,1))
        fprintf('%4d  %8.0f   %8.3f   %9.3f  %9.3f\n', j-1, R(j,:));
    end
end

fprintf('\n onset delay [ms]   (+ = LAGGING GT, - = leading)\n');
fprintf(' lag  delay[ms]');
for f = 1:nF, fprintf('     %.0f%%_median    n', brkFrac(f)*100); end
fprintf('\n');
for j = 1:nLag
    if ~isfinite(dLag(j)), continue; end
    fprintf('%4d  %8.0f', j-1, dLag(j));
    for f = 1:nF
        fprintf('  %+11.0f %4d', Dmed(j,f), nOn(j,f));
    end
    fprintf('\n');
end

%% ==================== FIGURE 1: v_x BY LAG ====================
cm = turbo(nLag);
figure('Color','w');
axLag = axes; hold on; grid on;
plot(gtTu, gtVu, 'k-', 'LineWidth', 0.8, 'DisplayName', 'GT');
for j = 1:nLag
    if isempty(SS{j}), continue; end
    x = SS{j}(:,1);  y = SS{j}(:,2);
    br = find(diff(x) > gapMax);
    for q = numel(br):-1:1
        x = [x(1:br(q)); NaN; x(br(q)+1:end)];
        y = [y(1:br(q)); NaN; y(br(q)+1:end)];
    end
    if j == 1, cc = [0.90 0.10 0.75]; lw = 1.8;
    else,      cc = cm(j,:);          lw = 1.0;  end
    plot(x, y, '-', 'Color', cc, 'LineWidth', lw, ...
         'DisplayName', sprintf('lag %d  (%.0f ms)', j-1, dLag(j)));
end
xlabel('time [s]'); ylabel('v_x [m/s]');
legend('Location', 'eastoutside'); title(sprintf('Per-lag smoothing - slot %d', sl));

%% ==================== FIGURE 2: REGIMES, EVENTS, AND GT ONSET ====================
isTr = abs(aGtCls) > aThr;
figure('Color','w');
axC1 = subplot(2,1,1); hold on; grid on;
plot(gtTu, gtVu, 'k-', 'DisplayName', 'GT');
plot(gtTu(isTr), gtVu(isTr), 'r.', 'MarkerSize', 8, 'DisplayName', 'transient');
for m = 1:size(ev,1)
    xline(ev(m,1), 'b:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    if isfinite(tBrkGt(m,1))
        xline(tBrkGt(m,1), 'g-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
end
ylabel('v_x [m/s]'); legend('Location', 'best');
title(sprintf('%d maneuvers   (blue = event, green = GT onset at %.0f%%)', ...
      size(ev,1), brkFrac(1)*100));

axC2 = subplot(2,1,2); hold on; grid on;
plot(gtTu, aGtCls, 'Color', [0.85 0.20 0.10], ...
     'DisplayName', sprintf('dv/dt smooth %.2fs (regime)', wSmooth));
plot(gtTu, aGtEv,  'Color', [0.10 0.55 0.85], 'LineWidth', 1.4, ...
     'DisplayName', sprintf('dv/dt smooth %.2fs (events)', wSmoothEv));
yline( aOn, 'g:', 'HandleVisibility', 'off');
yline(-aOn, 'g:', 'HandleVisibility', 'off');
yline( aThr, ':', 'HandleVisibility', 'off');
yline(-aThr, ':', 'HandleVisibility', 'off');
xlabel('time [s]'); ylabel('a_x [m/s^2]'); legend('Location', 'best');

%% ==================== FIGURE 3: RMSE BY REGIME ====================
figure('Color','w'); axR1 = axes; hold on; grid on;
plot(R(:,1), R(:,2), 'ko-', 'LineWidth', 1.2, 'DisplayName', 'all');
plot(R(:,1), R(:,3), 'o-',  'LineWidth', 1.6, 'Color', [0.10 0.55 0.25], ...
     'DisplayName', 'steady-state');
plot(R(:,1), R(:,4), 's-',  'LineWidth', 1.6, 'Color', [0.85 0.20 0.10], ...
     'DisplayName', 'transient');
xlabel('delay [ms]  (0 ms = lag 1)'); ylabel('RMSE v_x [m/s]');
legend('Location', 'best'); title('RMSE vs GT(t-d) by regime');

%% ==================== FIGURE 4: DELAY ANALYSIS ====================
figure('Color','w'); axA = axes; hold on; grid on;
cf = [0.00 0.45 0.74; 0.85 0.33 0.10];
for f = 1:nF
    plot(dLag, Dmed(:,f), 'o-', ...
        'LineWidth', 1.5, 'Color', cf(min(f,2),:), ...
        'DisplayName', sprintf('threshold %.0f%%', brkFrac(f)*100));
end
yline(0, 'k-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlabel('buffer age [ms]  (0 ms = lag 1)'); ylabel('onset delay [ms]   (+ = delay, - = lead)');
legend('Location', 'best');
title('Delay Analysis');

%% ==================== AXIS SYNCHRONIZATION ====================
linkaxes([axLag axC1 axC2], 'x');   % time axis
linkaxes([axR1 axA], 'x');          % delay/buffer-age axis


%% ==================== LOCAL FUNCTIONS ====================
function X = maskByStamps(X, nv)
% Set samples beyond the number of valid steps nv to NaN, row by row.
    for r = 1:size(X,1)
        if nv(r) >= 0 && nv(r) < size(X,2)
            X(r, nv(r)+1:end) = NaN;
        end
    end
end

function S = buildLagSeries(X, T, M, col, tnow)
% Sorted, unique [t, value] series for each lag.
    nL = size(M,2);
    S  = cell(nL,1);
    for j = 1:nL
        c = col(j);  ok = M(:,j) & isfinite(tnow);
        if nnz(ok) < 5, continue; end
        [tt, is] = sort(T(ok,c));  yy = X(ok,c);  yy = yy(is);
        [tt, iu] = unique(tt);     yy = yy(iu);
        S{j} = [tt yy];
    end
end

function ev = detectEvents(t, x, aOn, aOff, minGap)
% [start end] intervals with |x| > aOn, extended using aOff hysteresis.
    on = abs(x) > aOn;
    ev = [];
    k  = 1;
    while k <= numel(on)
        if on(k)
            s0 = k;
            while k <= numel(on) && abs(x(k)) > aOff, k = k + 1; end
            b = s0;
            while b > 1 && abs(x(b)) > aOff, b = b - 1; end
            kk = min(k, numel(t));
            if t(kk) - t(b) > 0.3
                ev(end+1,:) = [t(b) t(kk)]; %#ok<AGROW>
            end
        end
        k = k + 1;
    end
    if ~isempty(ev)
        keep = true(size(ev,1),1);
        for m = 2:size(ev,1)
            if ev(m,1) - ev(m-1,2) < minGap
                ev(m,1)   = ev(m-1,1);
                keep(m-1) = false;
            end
        end
        ev = ev(keep,:);
    end
end

function tb = breakVx(tu, y, v0, dirS, frac, nH)
% First instant when the deviation from the v0 plateau exceeds frac*|v0|
% in direction dirS and persists for nH consecutive samples.
    cnd = (y - v0) * dirS > frac * abs(v0);
    cnd(~isfinite(y)) = false;
    i = find(movsum(cnd, [0 nH-1]) == nH, 1);
    if isempty(i), tb = NaN; else, tb = tu(i); end
end

function [Q, n] = lagStats(D, nLag)
% Median for each column (lag), ignoring NaNs.
    Q = nan(nLag, 1);
    n = zeros(nLag, 1);
    for j = 1:nLag
        d = D(:,j);  d = d(isfinite(d));
        n(j) = numel(d);
        if ~isempty(d), Q(j) = median(d); end
    end
end
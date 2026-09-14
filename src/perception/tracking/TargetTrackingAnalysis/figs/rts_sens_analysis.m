%% ====================================================================
%  Lag/latency analysis of opponent tracking - v_x only
%
%  Break point (maneuver onset), based on velocity:
%    tbreak = min{ t : dir*(v(t) - v0) > frac*|v0|  sustained for holdT }
%    dOnset = tbreak,estimated - tbreak,GT     (+ = DELAY, - = lead)
%
%  FIXES vs the previous version
%  -----------------------------
%  (1) SYMMETRIC PLATEAU.  v0 is now computed independently on EACH series
%      over its OWN pre-event window.  Previously the GT plateau was reused
%      for the estimate, so any static bias of the estimate on the plateau
%      ate into the threshold budget and made the low-threshold detector
%      fire early in a systematic way.
%
%  (2) PERSISTENCE ON NATIVE SAMPLES.  The hold test used to run on the
%      uniform dtU grid: 4 interpolated samples on GT = 80 ms of real data,
%      but on a per-lag series with a ~60-130 ms native step the same 4
%      samples were the linear ramp between two real points, so a single
%      noisy sample passed the test.  The test now runs on the native
%      samples of each series over a real duration holdT, with at least
%      minPts real samples inside the window.  The returned instant is
%      refined by linear interpolation between the two samples straddling
%      the threshold, so it does not inherit the sampling step.
%
%  (3) REACHABLE THRESHOLDS.  Events whose GT excursion does not clear the
%      LARGEST threshold with margin are discarded, so every threshold sees
%      the same maneuver population (minDv alone did not guarantee this).
%
%  (4) THRESHOLD SWEEP.  brkFrac accepts any number of values: if the
%      measured delay grows with the threshold and then saturates, the
%      saturation value is the true group delay; a low threshold measures
%      "something moved", a high one measures "the amplitude was
%      recovered".  Default here is 2% and 5%.
%
%  (5) BUFFER-AGE ANCHOR.  rebaseLag1 = false by default: 0 ms is now lag 0,
%      the freshest, purely causal sample.  Older buffer entries have been
%      re-smoothed with later measurements, which is why they "lead" GT, so
%      lag 0 must be the point of MAXIMUM delay.  A curve that is not
%      monotonically decreasing with buffer age is a detector artifact, not
%      physics.  Set rebaseLag1 = true to restore the previous axis.
%
%  (6) TIME WINDOW SELECTION.  A new section lets you restrict the analysis
%      (maneuver events + RMSE) to a chosen time range, after showing you
%      the full list of detected maneuvers so you can pick one on data,
%      not by guessing. See "TIME WINDOW SELECTION" below.
%
%  Acceleration is used only as the smoothed derivative of v_x to segment
%  maneuvers.
% ====================================================================

%% ==================== PARAMETERS ====================
sl        = 1;                    % opponent slot
fld       = 'opponents__vx';      % velocity field in the buffer
nLag      = 15;                   % number of lags to analyze
gapMax    = 0.30;                 % s: gap above which plotted lines are split
useStamp  = false;                % true = lag 0 at the publication timestamp
dtU       = 0.02;                 % s: uniform working-grid step
rebaseLag1 = true;                % true = 0 ms is lag 1 (legacy axis)

% --- v_x derivative (maneuver segmentation) ---
wSmoothEv = 0.50;                 % s: smoothing for maneuver segmentation
aOn       = 1.0;                  % m/s^2: event activation threshold
aOff      = 0.5;                  % m/s^2: event deactivation threshold (hysteresis, aOff < aOn)
minGapEv  = 0.30;                 % s: closer events are merged

% --- onset based on v_x ---
brkFrac   = [0.02 0.05];          % break thresholds, fraction of the |v0| plateau
                                  % add e.g. 0.10 0.20 to check for saturation
holdT     = 0.08;                 % s: required persistence (60-100 ms)
minPts    = 2;                    % native samples required inside the hold window
preWin    = 1.00;                 % s: margin before the event, and plateau window
minPre    = 3;                    % native samples required in the plateau window
minDv     = 1.0;                  % m/s: minimum v_x change required to validate a maneuver
ampMargin = 1.5;                  % excursion must exceed ampMargin*max(brkFrac)*|v0|
useCommon = true;                 % true = statistics only on events valid for ALL lags

% --- time window (see "TIME WINDOW SELECTION" section below) ---
TIME_WINDOW = [500 1400];                 % e.g. [40 90]; relative seconds, same axis as
                                  % gtTu/plots. Empty = use the whole log.

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

if rebaseLag1 && nLag >= 2 && isfinite(dLag(2))
    dLag = dLag - dLag(2);              % legacy axis: 0 ms = lag 1
end
lagRefTxt = '0 ms = lag 0';
if rebaseLag1, lagRefTxt = '0 ms = lag 1'; end

SS = buildLagSeries(V, T, M, col, tnow);   % estimated v_x series, by lag

% native sampling step of each lag series (diagnostic: hold-test resolution)
dtNat = nan(nLag,1);
for j = 1:nLag
    if ~isempty(SS{j}) && size(SS{j},1) > 2, dtNat(j) = median(diff(SS{j}(:,1))); end
end

%% ==================== GROUND TRUTH ====================
gtT = double(gt.stamp) - t0;  gtV = double(gt.vx);
g   = isfinite(gtT) & isfinite(gtV);
[gtTu, iu] = unique(gtT(g));  gtVu = gtV(g);  gtVu = gtVu(iu);

dtGt  = median(diff(gtTu));
aGtEv = gradient(movmean(gtVu, max(3, round(wSmoothEv/dtGt))), gtTu);  % segmentation

%% ==================== MANEUVER EVENTS ====================
ev    = detectEvents(gtTu, aGtEv, aOn, aOff, minGapEv);
dirEv = nan(size(ev,1),1);
v0Ev  = nan(size(ev,1),1);

nRejAmp = 0;  nRejThr = 0;
if ~isempty(ev)
    keepEv = true(size(ev,1),1);
    for m = 1:size(ev,1)
        idx = gtTu >= ev(m,1) & gtTu <= ev(m,2);
        pre = gtTu >= ev(m,1) - preWin & gtTu < ev(m,1);
        if nnz(idx) < 3 || nnz(pre) < minPre
            keepEv(m) = false;  nRejAmp = nRejAmp + 1;  continue;
        end
        vEnd     = median(gtVu(gtTu > ev(m,2) - 0.2 & gtTu <= ev(m,2)), 'omitnan');
        v0Ev(m)  = median(gtVu(pre), 'omitnan');
        dirEv(m) = sign(vEnd - v0Ev(m));
        if ~isfinite(dirEv(m)) || dirEv(m) == 0 || ~isfinite(v0Ev(m))
            keepEv(m) = false;  nRejAmp = nRejAmp + 1;  continue;
        end
        % excursion must be large enough for EVERY threshold to be reachable
        exc = max(dirEv(m) * (gtVu(idx) - v0Ev(m)));
        if ~isfinite(exc) || exc < minDv
            keepEv(m) = false;  nRejAmp = nRejAmp + 1;  continue;
        end
        if exc < ampMargin * max(brkFrac) * abs(v0Ev(m))
            keepEv(m) = false;  nRejThr = nRejThr + 1;  continue;
        end
    end
    fprintf(' events detected: %d   discarded: %d (Delta v < %.1f m/s or bad direction), %d (excursion below %.0f%% threshold margin)\n', ...
            numel(keepEv), nRejAmp, minDv, nRejThr, max(brkFrac)*100);
    ev = ev(keepEv,:);  dirEv = dirEv(keepEv);  v0Ev = v0Ev(keepEv);
end

%% ==================== TIME WINDOW SELECTION ====================
% Shows every maneuver event survived the filters above, with its time
% span and direction, plus the full span of the log. Use this listing to
% choose TIME_WINDOW (set it in PARAMETERS, or edit it here and re-run
% from this section onward) before committing to the onset/RMSE analysis.
%
% Rationale: onset delay and RMSE are computed only from data inside
% TIME_WINDOW. This matters because different portions of a run can have
% very different tracking behaviour (e.g. a clean straight vs a chaotic
% overtake); mixing them into one median can hide or invent an effect
% that is really tied to one specific maneuver.
fprintf('\n=== AVAILABLE MANEUVER EVENTS (for TIME_WINDOW selection) ===\n');
fprintf('%4s %10s %10s %10s %8s\n', '#', 't_start[s]', 't_end[s]', 'dur[s]', 'dir');
for m = 1:size(ev,1)
    fprintf('%4d %10.2f %10.2f %10.2f %8+d\n', m, ev(m,1), ev(m,2), ev(m,2)-ev(m,1), dirEv(m));
end
fprintf('Full log span: [%.2f , %.2f] s\n', gtTu(1), gtTu(end));

if ~isempty(TIME_WINDOW)
    keepWin = ev(:,1) >= TIME_WINDOW(1) & ev(:,2) <= TIME_WINDOW(2);
    fprintf('TIME_WINDOW = [%.2f %.2f] s -> keeping %d of %d events\n', ...
        TIME_WINDOW(1), TIME_WINDOW(2), nnz(keepWin), numel(keepWin));
    ev = ev(keepWin,:);  dirEv = dirEv(keepWin);  v0Ev = v0Ev(keepWin);
else
    fprintf('TIME_WINDOW empty -> using the whole log (%d events).\n', size(ev,1));
end

nEv = size(ev,1);
if nEv == 0
    error('cpr: no maneuver events left after TIME_WINDOW - widen it or set it to [].');
end
nF  = numel(brkFrac);

%% ==================== ONSET BASED ON v_x ====================
tBrkGt  = nan(nEv, nF);
tBrkEst = nan(nEv, nLag, nF);
v0Est   = nan(nEv, nLag);        % per-series plateau (diagnostic: static bias)

for m = 1:nEv
    tA = ev(m,1) - preWin;  tB = ev(m,2);

    % --- GT, on its own native grid ---
    gsel = gtTu >= tA & gtTu <= tB;
    tg = gtTu(gsel);  yg = gtVu(gsel);
    if numel(tg) < 10, continue; end
    v0g = plateauV(tg, yg, tA, ev(m,1), minPre);
    if ~isfinite(v0g), continue; end
    for f = 1:nF
        tBrkGt(m,f) = breakVxT(tg, yg, v0g, dirEv(m), brkFrac(f), holdT, minPts, tA);
    end
    if all(isnan(tBrkGt(m,:))), continue; end

    % --- estimate, on its own native grid, with its own plateau ---
    for j = 1:nLag
        if isempty(SS{j}), continue; end
        tt = SS{j}(:,1);  yy = SS{j}(:,2);
        if tt(1) > tA || tt(end) < tB, continue; end          % window covered
        esel = tt >= tA & tt <= tB;
        te = tt(esel);  ye = yy(esel);
        if numel(te) < max(6, minPre + minPts), continue; end
        v0e = plateauV(te, ye, tA, ev(m,1), minPre);
        if ~isfinite(v0e), continue; end
        v0Est(m,j) = v0e;
        for f = 1:nF
            tBrkEst(m,j,f) = breakVxT(te, ye, v0e, dirEv(m), brkFrac(f), holdT, minPts, tA);
        end
    end
end

dOnset = (tBrkEst - reshape(tBrkGt, nEv, 1, nF)) * 1e3;   % ms; + = delay, - = lead

%% ==================== COMMON EVENT SET ====================
% Without this, each lag would have a different maneuver population and
% the medians would not be comparable.
okEv = reshape(all(isfinite(dOnset), 2), nEv, nF);
if useCommon
    for f = 1:nF
        dOnset(~okEv(:,f), :, f) = NaN;
    end
end

Dmed = nan(nLag, nF);
nOn  = zeros(nLag, nF);
for f = 1:nF
    [Dmed(:,f), nOn(:,f)] = lagStats(dOnset(:,:,f), nLag);
end

% static plateau bias of the estimate, in units of the smallest threshold:
% if this is not << 1 the low-threshold onset is not trustworthy
biasV = nan(nLag,1);
for j = 1:nLag
    d = dirEv .* (v0Est(:,j) - v0Ev);
    d = d(isfinite(d));
    if ~isempty(d), biasV(j) = median(d); end
end

%% ==================== RMSE vs GT(t-d) ====================
common = all(M, 2) & isfinite(tnow);
if ~isempty(TIME_WINDOW)
    common = common & tnow >= TIME_WINDOW(1) & tnow <= TIME_WINDOW(2);
end
for j = 1:nLag
    c = col(j);
    common = common & T(:,c) >= gtTu(1) & T(:,c) <= gtTu(end);
end

R = nan(nLag, 2);   % [buffer age, RMSE]
for j = 1:nLag
    c = col(j);
    if nnz(common) < 10, continue; end
    tt = T(common,c);
    e  = V(common,c) - interp1(gtTu, gtVu, tt, 'linear', NaN);
    R(j,:) = [dLag(j), sqrt(mean(e.^2, 'omitnan'))];
end

%% ==================== OUTPUT ====================
fprintf('\n onset based on v_x   plateau thresholds %s%%, hold %.0f ms on native samples (min %d)\n', ...
        mat2str(brkFrac*100), holdT*1e3, minPts);
if ~isempty(TIME_WINDOW)
    fprintf(' time window = [%.2f %.2f] s\n', TIME_WINDOW(1), TIME_WINDOW(2));
end
fprintf(' common samples = %d\n', nnz(common));
fprintf(' valid maneuvers = %d   (events common to all lags, per threshold: %s)\n\n', ...
        nEv, mat2str(sum(okEv,1)));

fprintf(' lag  bufAge[ms]      RMSE   dtNative[ms]   plateauBias[m/s]\n');
fprintf('      (%s)\n', lagRefTxt);
for j = 1:nLag
    if isfinite(R(j,1))
        fprintf('%4d  %9.0f  %8.3f  %10.0f  %15.3f\n', ...
                j-1, R(j,1), R(j,2), dtNat(j)*1e3, biasV(j));
    end
end

fprintf('\n onset delay [ms]   (+ = LAGGING GT, - = leading)\n');
fprintf(' lag  bufAge[ms]');
for f = 1:nF, fprintf('   %4.0f%%_med    n', brkFrac(f)*100); end
fprintf('\n');
for j = 1:nLag
    if ~isfinite(dLag(j)), continue; end
    fprintf('%4d  %9.0f', j-1, dLag(j));
    for f = 1:nF
        fprintf('  %+9.0f %4d', Dmed(j,f), nOn(j,f));
    end
    fprintf('\n');
end
fprintf('\n If the per-threshold delay grows with the threshold and then saturates,\n');
fprintf(' the saturation value is the true group delay.\n');

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
if ~isempty(TIME_WINDOW)
    xline(TIME_WINDOW(1), 'k--', 'HandleVisibility', 'off');
    xline(TIME_WINDOW(2), 'k--', 'HandleVisibility', 'off');
end
xlabel('time [s]'); ylabel('v_x [m/s]');
legend('Location', 'eastoutside'); title(sprintf('Per-lag smoothing - slot %d', sl));

%% ==================== FIGURE 2: EVENTS AND GT ONSET ====================
figure('Color','w');
axC1 = subplot(2,1,1); hold on; grid on;
plot(gtTu, gtVu, 'k-', 'DisplayName', 'GT');
for m = 1:nEv
    xline(ev(m,1), 'b:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    if isfinite(tBrkGt(m,1))
        xline(tBrkGt(m,1), 'g-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
end
ylabel('v_x [m/s]'); legend('Location', 'best');
title(sprintf('%d maneuvers   (blue = event, green = GT onset at %.0f%%)', ...
      nEv, brkFrac(1)*100));

axC2 = subplot(2,1,2); hold on; grid on;
plot(gtTu, aGtEv, 'Color', [0.10 0.55 0.85], 'LineWidth', 1.4, ...
     'DisplayName', sprintf('dv/dt smooth %.2fs (events)', wSmoothEv));
yline( aOn, 'g:', 'HandleVisibility', 'off');
yline(-aOn, 'g:', 'HandleVisibility', 'off');
xlabel('time [s]'); ylabel('a_x [m/s^2]'); legend('Location', 'best');

%% ==================== FIGURE 3: RMSE ====================
figure('Color','w'); axR1 = axes; hold on; grid on;
plot(R(:,1), R(:,2), 'ko-', 'LineWidth', 1.2);
xlabel(sprintf('buffer age [ms]  (%s)', lagRefTxt)); ylabel('RMSE v_x [m/s]');
title('RMSE vs GT(t-d)');

%% ==================== FIGURE 4: DELAY ANALYSIS ====================
figure('Color','w'); axA = axes; hold on; grid on;
cf = [0.00 0.45 0.74; 0.85 0.33 0.10; lines(max(0, nF-2))];
for f = 1:nF
    plot(dLag, Dmed(:,f), 'o-', 'LineWidth', 1.4, 'Color', cf(f,:), ...
        'DisplayName', sprintf('threshold %.0f%%', brkFrac(f)*100));
end
yline(0, 'k-', 'LineWidth', 0.8, 'HandleVisibility', 'off');
xlabel(sprintf('buffer age [ms]  (%s)', lagRefTxt));
ylabel('onset delay [ms]   (+ = delay, - = lead)');
legend('Location', 'best');
title('Delay Analysis');

%% ==================== AXIS SYNCHRONIZATION ====================
linkaxes([axLag axC1 axC2], 'x');   % time axis
linkaxes([axR1 axA], 'x');          % buffer-age axis


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

function v0 = plateauV(t, y, tA, tB, minPre)
% Pre-event plateau of a series, on its OWN samples in [tA, tB).
    v0  = NaN;
    idx = t >= tA & t < tB & isfinite(y);
    if nnz(idx) >= minPre, v0 = median(y(idx), 'omitnan'); end
end

function tb = breakVxT(t, y, v0, dirS, frac, holdT, minPts, tStart)
% First instant when the deviation from the v0 plateau exceeds frac*|v0| in
% direction dirS and stays above for holdT of REAL time, checked on the
% native samples of the series, with at least minPts samples inside the
% window.  The instant is refined by linear interpolation between the two
% samples straddling the threshold, so it is independent of the sampling
% step of the series (GT and estimate are therefore comparable even though
% their native rates differ by an order of magnitude).
    tb  = NaN;
    lev = frac * abs(v0);
    d   = (y - v0) * dirS;
    n   = numel(t);
    for i = 1:n
        if ~isfinite(d(i)) || d(i) <= lev || t(i) < tStart, continue; end
        jEnd = find(t <= t(i) + holdT, 1, 'last');
        jEnd = max(jEnd, min(i + minPts - 1, n));
        if jEnd >= n && t(n) < t(i) + holdT
            return;                       % not enough data left to confirm
        end
        seg = d(i:jEnd);
        if all(isfinite(seg)) && all(seg > lev)
            if i > 1 && isfinite(d(i-1)) && d(i-1) <= lev && d(i) > d(i-1)
                tb = t(i-1) + (lev - d(i-1)) * (t(i) - t(i-1)) / (d(i) - d(i-1));
            else
                tb = t(i);
            end
            return;
        end
    end
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
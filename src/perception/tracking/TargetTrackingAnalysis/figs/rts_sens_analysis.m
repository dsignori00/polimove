%% ====================================================================
%  Lag/latency analysis of opponent tracking - v_x only
%
%  WHAT CHANGED IN THIS VERSION
%  ----------------------------
%  (A) LAG SELECTION.  The analysis horizon is now set on the buffer-age
%      axis (maxAge, default 500 ms): every lag whose buffer age is
%      <= maxAge is used for the RMSE and delay figures (full resolution).
%      The v_x figures draw only lag 0, lag 1 and then one lag every
%      ~plotStep ms (default 100 ms) up to maxAge -> ~7 lines.
%
%  (B) ONSET = KNEE OF A TWO-SEGMENT FIT (primary method).
%      On each series (GT and every lag, each on its own native samples)
%      a continuous piecewise-linear model is fitted over the SAME time
%      window:
%            v(t) = a + b*min(t-tk,0) + c*max(t-tk,0)
%      tk (the knee) is found by a grid search + parabolic refinement.
%      The knee is the intersection of the "before" line and the "after"
%      ramp, so it does not depend on the absolute speed (no % of |v0|),
%      on the noise on the plateau, or on how rounded the corner is: a
%      symmetric smoother that rounds the corner on both sides keeps the
%      knee in place, a smoother that "anticipates" moves it earlier
%      (negative delay), a causal filter moves it later (positive delay).
%      The pre-segment has its own slope, so braking that starts while
%      the car is still accelerating (peaks) is handled too.
%
%  (C) SHIFT ALIGNMENT OF THE TRANSITION (secondary, cross-check).
%      Delay d minimizing  mean( (v_est(t) - v_gt(t-d))^2 )  over the
%      transition window, after removing each series' own plateau level.
%      It measures the time shift of the whole braking ramp.  If knee and
%      shift agree, the result is solid; if they disagree, the estimate
%      changes SHAPE (not only timing) and that event deserves a look.
%
%  (D) BRAKING ONLY by default (brakingOnly = true).
%
%  (E) CONSISTENCY.  Medians are shown with the inter-quartile band, a
%      per-event table is printed, and a diagnostic figure shows the fit
%      on one chosen event (showEv) so the detected knees can be checked
%      by eye.
%
%  (F) ONSET = DEPARTURE FROM THE PRE-MANEUVER TREND (primary for "when
%      does the maneuver start").  On each series a line is fitted on its
%      own samples BEFORE the maneuver; the onset is the first instant the
%      series drops below that line by onLev m/s (same absolute level for
%      GT and every lag) and stays there for onHold.  Unlike the shift, it
%      looks only at the START, not at how steep the ramp is afterwards;
%      unlike the old % thresholds, it follows the pre-trend (so braking
%      from a peak works) and uses small absolute levels.  Several levels
%      are shown: if the delay is the same at all levels, the start is a
%      clean time shift.
%
%  The old threshold method (brkFrac of |v0|) is kept for comparison only
%  (showThresh = true).
% ====================================================================

%% ==================== PARAMETERS ====================
sl         = 1;                   % opponent slot
fld        = 'opponents__vx';     % velocity field in the buffer
maxAge     = 500;                 % ms: analysis horizon on the buffer-age axis
ageTol     = 25;                  % ms: tolerance when picking the last lag
plotStep   = 100;                 % ms: spacing of the lags drawn in the v_x figures
gapMax     = 0.30;                % s: gap above which plotted lines are split
useStamp   = false;               % true = lag 0 at the publication timestamp
rebaseLag1 = true;                % true = 0 ms is lag 1 (legacy axis)

% --- v_x derivative (maneuver segmentation) ---
wSmoothEv  = 0.50;                % s: smoothing for maneuver segmentation
aOn        = 1.0;                 % m/s^2: event activation threshold
aOff       = 0.5;                 % m/s^2: event deactivation threshold (hysteresis)
minGapEv   = 0.30;                % s: closer events are merged
preWin     = 1.00;                % s: plateau window used to validate events
minDv      = 1.0;                 % m/s: minimum v_x change to validate a maneuver
brakingOnly = true;               % true = keep only braking maneuvers

% --- onset: knee of the two-segment fit (primary) ---
kPre       = 0.60;                % s: fit window before the coarse GT knee
kPost      = 0.45;                % s: fit window after the coarse GT knee
maxShift   = 0.20;                % s: knee search range around the GT knee
dtGrid     = 0.002;               % s: search grid (knee and shift)
minSide    = 3;                   % native samples required on each side of the knee
minSlopeChg = 3.0;                % m/s^2: minimum slope change to accept a knee

% --- delay by shift alignment (secondary) ---
xcMax      = 0.30;                % s: max |shift| searched

% --- onset = departure from the pre-maneuver trend (PRIMARY) ---
onLev      = [0.5 1.0];            % m/s: departure levels below the pre-trend (same for all series)
trGap      = 0.25;                % s: pre-trend fitted on [kneeGT-kPre, kneeGT-trGap]
onHold     = 0.06;                % s: persistence required below the level
onMinPts   = 2;                   % native samples inside the persistence window
showRamp   = false;               % extra figure: ramp delay (shift) by group

% --- legacy threshold onset (comparison only) ---
showThresh = false;              % measures shape, not time: off by default
brkFrac    = [0.02 0.05];         % fraction of the |v0| plateau
holdT      = 0.08;                % s: required persistence
minPts     = 2;                   % native samples inside the hold window
ampMargin  = 1.5;                 % excursion must exceed ampMargin*max(brkFrac)*|v0|
minPre     = 3;                   % native samples required in a plateau window

useCommon  = true;                % statistics only on events valid on most lags
minLagFrac = 0.90;                % ...i.e. valid on at least this fraction of lags
refLag     = 1;                   % lag index (0-based) used as reference for the
                                  % paired "smoothing gain" delay(lag) - delay(refLag)
showPerEv  = true;                % thin per-event lines in the delay figure
showGain   = false;               % extra figure: paired gain vs refLag (NOT vs GT)
showEv     = 1;                   % event shown in the diagnostic figure (0 = none)

% --- time window: relative seconds, same axis as the plots. [] = whole log ---
TIME_WINDOW = [500 1400];

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

%% ==================== LAG MASKS (all buffer depths) ====================
N    = size(V,2);
col  = N - (0:N-1);
M    = false(size(V,1), N);
L    = nan(size(V,1), N);

for j = 1:N
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

dLag = nan(N,1);
for j = 1:N
    if any(M(:,j)), dLag(j) = median(L(M(:,j), j)) * 1e3; end
end
if rebaseLag1 && N >= 2 && isfinite(dLag(2))
    dLag = dLag - dLag(2);              % legacy axis: 0 ms = lag 1
end
lagRefTxt = '0 ms = lag 0';
if rebaseLag1, lagRefTxt = '0 ms = lag 1'; end

% --- analysis horizon: every lag up to maxAge ---
nLag = find(isfinite(dLag) & dLag <= maxAge + ageTol, 1, 'last');
if isempty(nLag), error('cpr: no lag within maxAge = %.0f ms.', maxAge); end
if dLag(nLag) < maxAge - 2*ageTol
    warning('cpr: buffer reaches only %.0f ms (requested %.0f ms).', dLag(nLag), maxAge);
end
col = col(1:nLag);  M = M(:,1:nLag);  L = L(:,1:nLag);  dLag = dLag(1:nLag);

% --- lags drawn in the v_x figures: lag 0, lag 1, then every ~plotStep ms ---
plotLags = [1 2];
for tg = plotStep:plotStep:maxAge
    cand = 3:nLag;
    if isempty(cand), break; end
    [dm, k] = min(abs(dLag(cand) - tg));
    if dm <= plotStep/2, plotLags(end+1) = cand(k); end %#ok<AGROW>
end
plotLags = unique(plotLags(plotLags <= nLag), 'stable');
% Okabe-Ito palette (colour-blind safe, distinct on white). Yellow dropped
% (unreadable on white). lag 0 and lag 1 keep the two strongest colours;
% older lags cycle through the rest, then switch to dashed lines if needed.
okabe = [0.835 0.369 0.000;   % vermillion   -> lag 0
         0.000 0.447 0.698;   % blue         -> lag 1
         0.337 0.706 0.914;   % sky blue
         0.000 0.620 0.451;   % bluish green
         0.902 0.624 0.000;   % orange
         0.800 0.475 0.655;   % reddish purple
         0.494 0.247 0.698];  % violet (distinct from GT black)
cPl  = zeros(numel(plotLags), 3);
lsPl = repmat({'-'}, numel(plotLags), 1);
for q = 1:numel(plotLags)
    cPl(q,:) = okabe(mod(q-1, size(okabe,1)) + 1, :);
    if q > size(okabe,1), lsPl{q} = '--'; end
end
lwPl = 1.1 * ones(numel(plotLags), 1);  lwPl(plotLags == 1) = 1.8;  lwPl(plotLags == 2) = 1.4;

SS = buildLagSeries(V, T, M, col, tnow);   % estimated v_x series, by lag

dtNat = nan(nLag,1);
for j = 1:nLag
    if ~isempty(SS{j}) && size(SS{j},1) > 2, dtNat(j) = median(diff(SS{j}(:,1))); end
end

%% ==================== GROUND TRUTH ====================
gtT = double(gt.stamp(:)) - t0;  gtV = double(gt.vx(:));
g   = isfinite(gtT) & isfinite(gtV);
[gtTu, iu] = unique(gtT(g));  gtVu = gtV(g);  gtVu = gtVu(iu);
gtTu = gtTu(:);  gtVu = gtVu(:);

dtGt  = median(diff(gtTu));
aGtEv = gradient(movmean(gtVu, max(3, round(wSmoothEv/dtGt))), gtTu);

%% ==================== MANEUVER EVENTS ====================
ev    = detectEvents(gtTu, aGtEv, aOn, aOff, minGapEv);
dirEv = nan(size(ev,1),1);
v0Ev  = nan(size(ev,1),1);

nRejAmp = 0;  nRejThr = 0;  nRejDir = 0;
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
        if brakingOnly && dirEv(m) > 0
            keepEv(m) = false;  nRejDir = nRejDir + 1;  continue;
        end
        exc = max(dirEv(m) * (gtVu(idx) - v0Ev(m)));
        if ~isfinite(exc) || exc < minDv
            keepEv(m) = false;  nRejAmp = nRejAmp + 1;  continue;
        end
        if showThresh && exc < ampMargin * max(brkFrac) * abs(v0Ev(m))
            keepEv(m) = false;  nRejThr = nRejThr + 1;  continue;
        end
    end
    fprintf(' events detected: %d   discarded: %d (Delta v / plateau), %d (not braking), %d (threshold margin)\n', ...
            numel(keepEv), nRejAmp, nRejDir, nRejThr);
    ev = ev(keepEv,:);  dirEv = dirEv(keepEv);  v0Ev = v0Ev(keepEv);
end

%% ==================== TIME WINDOW SELECTION ====================
fprintf('\n=== AVAILABLE MANEUVER EVENTS (for TIME_WINDOW selection) ===\n');
fprintf('%4s %10s %10s %10s %8s\n', '#', 't_start[s]', 't_end[s]', 'dur[s]', 'dir');
for m = 1:size(ev,1)
    fprintf('%4d %10.2f %10.2f %10.2f %+8d\n', m, ev(m,1), ev(m,2), ev(m,2)-ev(m,1), dirEv(m));
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
nF = numel(brkFrac);

%% ==================== ONSET: KNEE FIT + SHIFT ALIGNMENT ====================
tkGt    = nan(nEv,1);
wEv     = nan(nEv,2);            % common fit window per event
pGt     = nan(nEv,3);
tkEst   = nan(nEv,nLag);
pEst    = nan(nEv,nLag,3);
dShift  = nan(nEv,nLag);
gapEv   = nan(nEv,nLag);         % max native gap inside the fit window [s]
tBrkGt  = nan(nEv,nF);
tBrkEst = nan(nEv,nLag,nF);
nL      = numel(onLev);
tOnGt   = nan(nEv,nL);
tOnEst  = nan(nEv,nLag,nL);

for m = 1:nEv
    e1 = ev(m,1);  dS = dirEv(m);

    % pass 1: coarse GT knee near the event start
    tk1 = kneeFit(gtTu, gtVu, e1 - 0.3 - kPre, e1 + 0.6 + kPost, e1 - 0.3, e1 + 0.6, ...
                  dS, dtGrid, minSide, minSlopeChg);
    if ~isfinite(tk1), continue; end

    % pass 2: SAME window and search range for GT and all lags
    wLo = tk1 - kPre;       wHi = tk1 + kPost;
    sLo = tk1 - maxShift;   sHi = tk1 + maxShift;
    [tkGt(m), pGt(m,:)] = kneeFit(gtTu, gtVu, wLo, wHi, sLo, sHi, dS, dtGrid, minSide, minSlopeChg);
    if ~isfinite(tkGt(m)), continue; end
    wEv(m,:) = [wLo wHi];

    gsub = gtTu >= wLo - xcMax - 0.1 & gtTu <= wHi + xcMax + 0.6;
    tgS  = gtTu(gsub);  ygS = gtVu(gsub);
    bG   = plateauV(tgS, ygS, wLo, sLo, minPre);

    tFa = tk1 - kPre;  tFb = tk1 - trGap;  tSe = wHi + 0.3;
    for l = 1:nL
        tOnGt(m,l) = onsetDepart(tgS, ygS, tFa, tFb, tSe, dS, onLev(l), onHold, onMinPts, minPre);
    end

    if showThresh && isfinite(bG)
        gs = tgS >= wLo & tgS <= wHi + 0.5;
        for f = 1:nF
            tBrkGt(m,f) = breakVxT(tgS(gs), ygS(gs), bG, dS, brkFrac(f), holdT, minPts, sLo);
        end
    end

    for j = 1:nLag
        if isempty(SS{j}), continue; end
        tt = SS{j}(:,1);  yy = SS{j}(:,2);
        in = tt >= wLo & tt <= wHi;
        if tt(1) > wLo || tt(end) < wHi || nnz(in) < 2*minSide + 1, continue; end
        gapEv(m,j) = max(diff([wLo; tt(in); wHi]));
        if gapEv(m,j) > gapMax, continue; end                 % hole inside the window

        [tkEst(m,j), p] = kneeFit(tt, yy, wLo, wHi, sLo, sHi, dS, dtGrid, minSide, minSlopeChg);
        pEst(m,j,:) = p;

        bE = plateauV(tt, yy, wLo, sLo, minPre);
        if isfinite(bE) && isfinite(bG)
            dShift(m,j) = shiftAlign(tt, yy - bE, tgS, ygS - bG, sLo, wHi, xcMax, dtGrid);
        end
        for l = 1:nL
            tOnEst(m,j,l) = onsetDepart(tt, yy, tFa, tFb, tSe, dS, onLev(l), onHold, onMinPts, minPre);
        end

        if showThresh && isfinite(bE)
            es = tt >= wLo & tt <= wHi + 0.5;
            for f = 1:nF
                tBrkEst(m,j,f) = breakVxT(tt(es), yy(es), bE, dS, brkFrac(f), holdT, minPts, sLo);
            end
        end
    end
end

dKnee  = (tkEst - tkGt) * 1e3;                     % ms; + = delay, - = lead
dShift = dShift * 1e3;                             % ms; + = delay, - = lead
dThr   = (tBrkEst - reshape(tBrkGt, nEv, 1, nF)) * 1e3;
dOn    = (tOnEst - reshape(tOnGt, nEv, 1, nL)) * 1e3;   % ms; + = late start, - = early

%% ==================== STATISTICS (common event set) ====================
jRef   = min(refLag + 1, nLag);
dGain  = dShift - dShift(:, jRef);                 % paired: removes the per-event offset
D     = {dKnee, dShift, dGain};
names = {'knee', 'shift', sprintf('shift - lag%d', jRef-1)};
if showThresh
    for f = 1:nF
        D{end+1}     = dThr(:,:,f);                         %#ok<AGROW>
        names{end+1} = sprintf('thr %.0f%%', brkFrac(f)*100); %#ok<AGROW>
    end
end
kOn = numel(D) + (1:nL);
for l = 1:nL
    D{end+1}     = dOn(:,:,l);                               %#ok<AGROW>
    names{end+1} = sprintf('onset %.1f m/s', onLev(l));      %#ok<AGROW>
end
nMeth = numel(D);
nCom  = zeros(1, nMeth);
Med = nan(nLag, nMeth);  Q1 = Med;  Q3 = Med;  Nn = zeros(nLag, nMeth);
okEv = false(nEv, nMeth);
for k = 1:nMeth
    okE = mean(isfinite(D{k}), 2) >= minLagFrac;
    if k == 3, okE = okE & isfinite(dShift(:, jRef)); end   % gain needs its reference
    okEv(:,k) = okE;
    nCom(k) = nnz(okE);
    if useCommon, D{k}(~okE,:) = NaN; end
    [Med(:,k), Nn(:,k), Q1(:,k), Q3(:,k)] = lagStats(D{k});
end
if useCommon && nCom(2) < 3
    warning('cpr: only %d events valid on >= %.0f%% of lags (shift).', nCom(2), minLagFrac*100);
end

%% ==================== ABSOLUTE DELAY vs GT: GROUPS + ZERO CROSSING ====================
% Each event is assigned to a group by the sign of its saturated delay
% (median of the oldest 25% of lags): group 1 ends LEADING GT, group 2
% stays LAGGING GT even at the oldest lag.  The zero crossing is the buffer
% age at which the estimate is aligned with GT (delay = 0).
DA    = D{2};                                   % shift vs GT, common-set filtered
nTail = max(1, round(0.25*nLag));
satV  = nan(nEv,1);  tZero = nan(nEv,1);  grp = nan(nEv,1);
for m = 1:nEv
    if ~okEv(m,2), continue; end
    y    = DA(m,:).';
    tail = y(end-nTail+1:end);  tail = tail(isfinite(tail));
    if isempty(tail), continue; end
    satV(m)  = median(tail);
    tZero(m) = zeroCross(dLag, y);
    grp(m)   = 1 + (satV(m) >= 0);
end
grpName = {'ends LEADING GT', 'stays LAGGING GT'};
MedG = nan(nLag,2);  Q1G = MedG;  Q3G = MedG;  nG = zeros(1,2);  tZeroG = nan(1,2);
for gI = 1:2
    sel = grp == gI;  nG(gI) = nnz(sel);
    if nG(gI) == 0, continue; end
    [MedG(:,gI), ~, Q1G(:,gI), Q3G(:,gI)] = lagStats(DA(sel,:));
    tZeroG(gI) = zeroCross(dLag, MedG(:,gI));
end
tZeroAll = zeroCross(dLag, Med(:,2));

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
fprintf('\n analysis horizon = %.0f ms  ->  %d lags   (%s)\n', maxAge, nLag, lagRefTxt);
if ~isempty(TIME_WINDOW)
    fprintf(' time window = [%.2f %.2f] s\n', TIME_WINDOW(1), TIME_WINDOW(2));
end
fprintf(' common samples (RMSE) = %d\n', nnz(common));
fprintf(' maneuvers = %d   events valid for ALL lags: %s = %s\n\n', ...
        nEv, strjoin(names, ' / '), mat2str(nCom));

fprintf(' lag  bufAge[ms]      RMSE   dtNative[ms]\n');
for j = 1:nLag
    if isfinite(R(j,1))
        fprintf('%4d  %9.0f  %8.3f  %10.0f\n', j-1, R(j,1), R(j,2), dtNat(j)*1e3);
    end
end

fprintf('\n onset delay [ms]  median [IQR]   (+ = LAGGING GT, - = leading)\n');
fprintf(' lag  bufAge[ms]');
for k = 1:nMeth, fprintf('  %22s', names{k}); end
fprintf('\n');
for j = 1:nLag
    if ~isfinite(dLag(j)), continue; end
    fprintf('%4d  %9.0f', j-1, dLag(j));
    for k = 1:nMeth
        fprintf('  %+6.0f [%+5.0f %+5.0f] n%-3d', Med(j,k), Q1(j,k), Q3(j,k), Nn(j,k));
    end
    fprintf('\n');
end

fprintf('\n per-event delay [ms]  knee | shift   (before common-set filtering)\n');
fprintf(' gap = largest hole between native samples inside the fit window [ms]\n');
fprintf('%4s %9s %15s %15s %15s %9s %9s %9s %6s\n', '#', 'tkGT[s]', 'lag 0', 'lag 1', ...
        sprintf('lag %d', nLag-1), 'gain[ms]', 'gap0', 'gap1', 'used');
for m = 1:nEv
    if ~isfinite(tkGt(m)), fprintf('%4d   GT knee not found\n', m); continue; end
    fprintf('%4d %9.2f   %+5.0f | %+5.0f   %+5.0f | %+5.0f   %+5.0f | %+5.0f %+9.0f %9.0f %9.0f %6s\n', ...
            m, tkGt(m), dKnee(m,1), dShift(m,1), dKnee(m,min(2,nLag)), dShift(m,min(2,nLag)), ...
            dKnee(m,nLag), dShift(m,nLag), dGain(m,nLag), gapEv(m,1)*1e3, gapEv(m,min(2,nLag))*1e3, ...
            mat2str(okEv(m,2)));
end
fprintf('\n knee and shift should agree; a large disagreement on one event means the\n');
fprintf(' estimate changes shape there (check it with showEv).\n');
fprintf(' gain = shift(lag %d) - shift(lag %d): delay removed by the smoother, offset-free.\n', ...
        nLag-1, jRef-1);

lMid = ceil(nL/2);
fprintf('\n ONSET = departure from the pre-trend, delay vs GT [ms]  median [IQR]  (+ = late, - = early)\n');
fprintf(' lag  bufAge[ms]');
for l = 1:nL, fprintf('  %22s', names{kOn(l)}); end
fprintf('\n');
for j = 1:nLag
    if ~isfinite(dLag(j)), continue; end
    fprintf('%4d  %9.0f', j-1, dLag(j));
    for l = 1:nL
        k = kOn(l);
        fprintf('  %+6.0f [%+5.0f %+5.0f] n%-3d', Med(j,k), Q1(j,k), Q3(j,k), Nn(j,k));
    end
    fprintf('\n');
end
fprintf('\n per-event ONSET delay [ms] at %.1f m/s\n', onLev(lMid));
fprintf('%4s %10s %8s %8s %8s\n', '#', 'tOnGT[s]', 'lag 0', 'lag 1', sprintf('lag %d', nLag-1));
for m = 1:nEv
    if ~isfinite(tOnGt(m,lMid)), fprintf('%4d   GT onset not found\n', m); continue; end
    fprintf('%4d %10.2f %+8.0f %+8.0f %+8.0f\n', m, tOnGt(m,lMid), ...
            dOn(m,1,lMid), dOn(m,min(2,nLag),lMid), dOn(m,nLag,lMid));
end
fprintf(' same delay at every level -> the start is a clean time shift;\n');
fprintf(' delay growing with the level -> the start is on time but the ramp is slower.\n');

fprintf('\n ABSOLUTE delay vs GT (shift) and zero crossing   (buffer age: %s)\n', lagRefTxt);
fprintf('%4s %6s %10s %10s %12s %12s\n', '#', 'group', 'lag0[ms]', 'lag1[ms]', 'satur.[ms]', 'zero@[ms]');
for m = 1:nEv
    if isnan(grp(m)), continue; end
    fprintf('%4d %6d %+10.0f %+10.0f %+12.0f %12s\n', m, grp(m), DA(m,1), DA(m,min(2,nLag)), ...
            satV(m), fmtZero(tZero(m)));
end
for gI = 1:2
    fprintf(' group %d (%s): n = %d, saturates at %+.0f ms, zero crossing at %s ms\n', ...
            gI, grpName{gI}, nG(gI), MedG(end,gI), fmtZero(tZeroG(gI)));
end
fprintf(' all events (median): zero crossing at %s ms\n', fmtZero(tZeroAll));
if rebaseLag1 && isfinite(dLag(1))
    fprintf(' NOTE: axis rebased on lag 1; add %.0f ms to read the age measured from lag 0\n', -dLag(1));
end

%% ==================== FIGURE 1: v_x BY LAG (selected lags) ====================
figure('Color','w');
axLag = axes; hold on; grid on;
plot(gtTu, gtVu, 'k-', 'LineWidth', 1.3, 'DisplayName', 'GT');
for q = 1:numel(plotLags)
    j = plotLags(q);
    if isempty(SS{j}), continue; end
    [x, y] = splitGaps(SS{j}(:,1), SS{j}(:,2), gapMax);
    plot(x, y, lsPl{q}, 'Color', cPl(q,:), 'LineWidth', lwPl(q), ...
         'DisplayName', sprintf('lag %d  (%.0f ms)', j-1, dLag(j)));
end
if ~isempty(TIME_WINDOW)
    xline(TIME_WINDOW(1), 'k--', 'HandleVisibility', 'off');
    xline(TIME_WINDOW(2), 'k--', 'HandleVisibility', 'off');
end
xlabel('time [s]'); ylabel('v_x [m/s]');
legend('Location', 'eastoutside'); title(sprintf('Per-lag smoothing - slot %d', sl));

%% ==================== FIGURE 2: EVENTS AND GT KNEE ====================
figure('Color','w');
axC1 = subplot(2,1,1); hold on; grid on;
plot(gtTu, gtVu, 'k-', 'DisplayName', 'GT');
for m = 1:nEv
    xline(ev(m,1), 'b:', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    if isfinite(tkGt(m))
        xline(tkGt(m), 'g-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
end
ylabel('v_x [m/s]'); legend('Location', 'best');
title(sprintf('%d maneuvers   (blue = event, green = GT knee)', nEv));

axC2 = subplot(2,1,2); hold on; grid on;
plot(gtTu, aGtEv, 'Color', [0.10 0.55 0.85], 'LineWidth', 1.4, ...
     'DisplayName', sprintf('dv/dt smooth %.2fs (events)', wSmoothEv));
yline( aOn, 'g:', 'HandleVisibility', 'off');
yline(-aOn, 'g:', 'HandleVisibility', 'off');
xlabel('time [s]'); ylabel('a_x [m/s^2]'); legend('Location', 'best');

%% ==================== FIGURE 3: RMSE (all lags) ====================
figure('Color','w'); axR1 = axes; hold on; grid on;
plot(R(:,1), R(:,2), 'ko-', 'LineWidth', 1.2);
xlabel(sprintf('buffer age [ms]  (%s)', lagRefTxt)); ylabel('RMSE v_x [m/s]');
title('RMSE vs GT(t-d)');

%% ==================== FIGURE 4: ONSET DELAY vs GT BY LAG ====================
cOn = [0.000 0.447 0.698; 0.835 0.369 0.000; lines(max(0, nL-2))];   % 0.5 m/s blue, 1.0 m/s vermillion
figure('Color','w'); axA = axes; hold on; grid on;
for l = 1:nL
    k  = kOn(l);
    ok = isfinite(dLag) & isfinite(Med(:,k));
    if nnz(ok) < 2, continue; end
    lw = 1.2;  if l == lMid, lw = 2.0; end
    plot(dLag(ok), Med(ok,k), 'o-', 'LineWidth', lw, 'Color', cOn(l,:), ...
         'DisplayName', sprintf('%s  (n=%d)', names{k}, nCom(k)));
end
yline(0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel(sprintf('buffer age [ms]  (%s)', lagRefTxt));
ylabel('onset delay vs GT [ms]   (+ = late start, - = early start)');
legend('Location', 'best');
title('Delay Analysis');

%% ==================== FIGURE 4b: RAMP DELAY (shift) BY GROUP (optional) ====================
cG = [0.000 0.447 0.698; 0.835 0.369 0.000];      % group 1 blue (lead), group 2 orange (lag)
axRamp = gobjects(0);
if showRamp
figure('Color','w'); axRamp = axes; hold on; grid on;
if showPerEv
    for m = find(isfinite(grp)).'
        ok = isfinite(dLag) & isfinite(DA(m,:)).';
        plot(dLag(ok), DA(m,ok), '-', 'Color', [cG(grp(m),:) 0.30], 'LineWidth', 0.7, ...
             'HandleVisibility', 'off');
        kl = find(ok, 1, 'last');
        text(dLag(kl), DA(m,kl), sprintf(' %d', m), 'Color', cG(grp(m),:), 'FontSize', 8);
    end
end
for gI = 1:2
    ok = isfinite(dLag) & isfinite(MedG(:,gI));
    if nnz(ok) < 2, continue; end
    x = dLag(ok);
    fill([x; flipud(x)], [Q1G(ok,gI); flipud(Q3G(ok,gI))], cG(gI,:), ...
         'FaceAlpha', 0.12, 'EdgeColor', 'none', 'HandleVisibility', 'off');
    plot(x, MedG(ok,gI), 'o-', 'LineWidth', 1.8, 'Color', cG(gI,:), ...
         'DisplayName', sprintf('group %d: %s  (n=%d)', gI, grpName{gI}, nG(gI)));
    if isfinite(tZeroG(gI))
        xline(tZeroG(gI), '--', sprintf('0 at %.0f ms', tZeroG(gI)), 'Color', cG(gI,:), ...
              'LabelVerticalAlignment', 'bottom', 'HandleVisibility', 'off');
    end
end
ok = isfinite(dLag) & isfinite(Med(:,2));
plot(dLag(ok), Med(ok,2), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.0, ...
     'DisplayName', sprintf('all events, median  (n=%d)', nCom(2)));
yline(0, 'k-', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel(sprintf('buffer age [ms]  (%s)', lagRefTxt));
ylabel('delay vs GT [ms]   (+ = lagging, - = leading)');
legend('Location', 'best');
title('Ramp delay (shift) vs GT by group - timing of the whole ramp, not of the start');
end

axA2 = gobjects(0);
if showGain
    figure('Color','w'); axA2 = axes; hold on; grid on;
    ok = isfinite(dLag) & isfinite(Med(:,3));
    if nnz(ok) > 1
        x = dLag(ok);
        fill([x; flipud(x)], [Q1(ok,3); flipud(Q3(ok,3))], [0.93 0.69 0.13], ...
             'FaceAlpha', 0.15, 'EdgeColor', 'none', 'HandleVisibility', 'off');
        plot(x, Med(ok,3), 'd-', 'LineWidth', 1.8, 'Color', [0.93 0.69 0.13], ...
             'DisplayName', sprintf('%s  (n=%d)', names{3}, nCom(3)));
    end
    yline(0, 'k-', 'HandleVisibility', 'off');
    xlabel(sprintf('buffer age [ms]  (%s)', lagRefTxt));
    ylabel(sprintf('delay vs lag %d [ms]  (NOT vs GT)', jRef-1));
    legend('Location', 'best');  title('Smoothing gain relative to lag 1');
end

%% ==================== FIGURE 5: KNEE DIAGNOSTIC ON ONE EVENT ====================
if showEv >= 1 && showEv <= nEv && isfinite(tkGt(showEv))
    m = showEv;  wLo = wEv(m,1);  wHi = wEv(m,2);
    tf = linspace(wLo, wHi, 300).';
    figure('Color','w'); hold on; grid on;
    gs = gtTu >= wLo - 0.3 & gtTu <= wHi + 0.3;
    plot(gtTu(gs), gtVu(gs), 'k.', 'MarkerSize', 5, 'DisplayName', 'GT');
    plot(tf, evalKnee(pGt(m,:), tkGt(m), tf), 'k--', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    xline(tkGt(m), 'k-', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    if isfinite(tOnGt(m,lMid))
        plot(tOnGt(m,lMid), interp1(gtTu, gtVu, tOnGt(m,lMid)), 'kv', 'MarkerFaceColor', 'k', ...
             'MarkerSize', 9, 'HandleVisibility', 'off');
    end
    for q = 1:numel(plotLags)
        j = plotLags(q);
        if isempty(SS{j}), continue; end
        s = SS{j}(:,1) >= wLo - 0.3 & SS{j}(:,1) <= wHi + 0.3;
        plot(SS{j}(s,1), SS{j}(s,2), '.-', 'Color', cPl(q,:), 'MarkerSize', 10, ...
             'DisplayName', sprintf('lag %d  (onset %+.0f ms)', j-1, dOn(m,j,lMid)));
        tOn = tOnEst(m,j,lMid);
        if isfinite(tOn)
            plot(tOn, interp1(SS{j}(:,1), SS{j}(:,2), tOn), 'v', 'MarkerFaceColor', cPl(q,:), ...
                 'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'HandleVisibility', 'off');
        end
        if isfinite(tkEst(m,j))
            plot(tf, evalKnee(squeeze(pEst(m,j,:)), tkEst(m,j), tf), '--', ...
                 'Color', cPl(q,:), 'HandleVisibility', 'off');
            xline(tkEst(m,j), ':', 'Color', cPl(q,:), 'LineWidth', 1.2, 'HandleVisibility', 'off');
        end
    end
    xline(wLo, 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    xline(wHi, 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
    xlim([wLo - 0.3, wHi + 0.3]);
    xlabel('time [s]'); ylabel('v_x [m/s]'); legend('Location', 'eastoutside');
    title(sprintf('Event %d - triangles = onset at %.1f m/s below pre-trend, dashed = knee fit', m, onLev(lMid)));
end

%% ==================== AXIS SYNCHRONIZATION ====================
linkaxes([axLag axC1 axC2], 'x');   % time axis
linkaxes([axR1 axA axA2 axRamp], 'x');     % buffer-age axis (axA2 empty if showGain = false)
xLagLim = [min(dLag, [], 'omitnan'), max(dLag, [], 'omitnan')];   % from the first to the last lag
if all(isfinite(xLagLim)) && diff(xLagLim) > 0
    xlim(axR1, xLagLim);
end


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
        S{j} = [tt(:) yy(:)];
    end
end

function [x, y] = splitGaps(x, y, gapMax)
% Insert NaNs where the time step exceeds gapMax (so lines are split).
    br = find(diff(x) > gapMax);
    for q = numel(br):-1:1
        x = [x(1:br(q)); NaN; x(br(q)+1:end)];
        y = [y(1:br(q)); NaN; y(br(q)+1:end)];
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
% Median level of a series on its OWN samples in [tA, tB).
    v0  = NaN;
    idx = t >= tA & t < tB & isfinite(y);
    if nnz(idx) >= minPre, v0 = median(y(idx), 'omitnan'); end
end

function [tk, p] = kneeFit(t, y, tLo, tHi, sLo, sHi, dirS, dtGrid, minSide, minChg)
% Continuous two-segment fit  v = a + b*min(t-tk,0) + c*max(t-tk,0)
% on the native samples in [tLo, tHi]; tk searched in [sLo, sHi].
% The knee is accepted only if the slope changes in the maneuver direction
% by at least minChg, has >= minSide samples on each side, and is a true
% interior minimum of the cost (not stuck on the search boundary).
    tk = NaN;  p = nan(3,1);
    s  = t >= tLo & t <= tHi & isfinite(y);
    t  = t(s);  y = y(s);  t = t(:);  y = y(:);
    if numel(t) < 2*minSide + 1, return; end
    sLo = max(sLo, tLo);  sHi = min(sHi, tHi);
    cand = sLo:dtGrid:sHi;
    if numel(cand) < 3, return; end
    cost = inf(size(cand));
    for k = 1:numel(cand)
        u = t - cand(k);
        if nnz(u < 0) < minSide || nnz(u > 0) < minSide, continue; end
        A = [ones(size(u)) min(u,0) max(u,0)];
        q = A \ y;
        if dirS*(q(3) - q(2)) < minChg, continue; end
        r = y - A*q;
        cost(k) = r.' * r;
    end
    tk = gridMin(cand, cost, dtGrid);
    if ~isfinite(tk), return; end
    u = t - tk;
    p = [ones(size(u)) min(u,0) max(u,0)] \ y;
end

function v = evalKnee(p, tk, t)
    u = t - tk;
    v = p(1) + p(2)*min(u,0) + p(3)*max(u,0);
end

function d = shiftAlign(te, ye, tg, yg, tLo, tHi, dMax, dtGrid)
% Shift d minimizing mean((ye(t) - yg(t-d))^2) over the estimate's native
% samples in [tLo, tHi].  + = estimate lags GT.  Inputs already de-biased.
    d = NaN;
    s = te >= tLo & te <= tHi & isfinite(ye);
    te = te(s);  ye = ye(s);
    if numel(te) < 5, return; end
    dd   = -dMax:dtGrid:dMax;
    cost = inf(size(dd));
    for k = 1:numel(dd)
        gg = interp1(tg, yg, te - dd(k), 'linear', NaN);
        if any(~isfinite(gg)), continue; end
        cost(k) = mean((ye - gg).^2);
    end
    d = gridMin(dd, cost, dtGrid);
end

function x = gridMin(xs, c, dx)
% Grid minimum with parabolic refinement; NaN if on the boundary.
    x = NaN;
    [cm, k] = min(c);
    if ~isfinite(cm) || k == 1 || k == numel(c), return; end
    off = 0;
    c1 = c(k-1);  c3 = c(k+1);
    if isfinite(c1) && isfinite(c3)
        den = c1 - 2*cm + c3;
        if den > 0, off = max(-1, min(1, 0.5*(c1 - c3)/den)); end
    end
    x = xs(k) + off*dx;
end

function tb = breakVxT(t, y, v0, dirS, frac, holdT, minPts, tStart)
% LEGACY threshold onset (comparison only): first instant when the
% deviation from v0 exceeds frac*|v0| in direction dirS and stays above
% for holdT on native samples; refined by linear interpolation.
    tb  = NaN;
    lev = frac * abs(v0);
    d   = (y - v0) * dirS;
    n   = numel(t);
    for i = 1:n
        if ~isfinite(d(i)) || d(i) <= lev || t(i) < tStart, continue; end
        jEnd = find(t <= t(i) + holdT, 1, 'last');
        jEnd = max(jEnd, min(i + minPts - 1, n));
        if jEnd >= n && t(n) < t(i) + holdT
            return;
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

function [Q, n, q25, q75] = lagStats(D)
% Median and inter-quartile range for each column (lag), ignoring NaNs.
    nL  = size(D,2);
    Q   = nan(nL,1);  q25 = Q;  q75 = Q;  n = zeros(nL,1);
    for j = 1:nL
        d = D(:,j);  d = d(isfinite(d));
        n(j) = numel(d);
        if ~isempty(d)
            Q(j) = median(d);  q25(j) = qtl(d, 0.25);  q75(j) = qtl(d, 0.75);
        end
    end
end

function q = qtl(d, p)
% Quantile by linear interpolation (no Statistics Toolbox needed).
    d = sort(d(:));  n = numel(d);
    if n == 0, q = NaN; return; end
    if n == 1, q = d;   return; end
    x  = 1 + (n-1)*p;  lo = floor(x);  hi = ceil(x);
    q  = d(lo) + (x - lo)*(d(hi) - d(lo));
end

function x0 = zeroCross(x, y)
% First crossing from + (lagging) to <= 0 (aligned/leading), linear interp.
    x0 = NaN;  x = x(:);  y = y(:);
    ok = isfinite(x) & isfinite(y);  x = x(ok);  y = y(ok);
    for i = 1:numel(y)-1
        if y(i) > 0 && y(i+1) <= 0
            x0 = x(i) + y(i) * (x(i+1) - x(i)) / (y(i) - y(i+1));
            return;
        end
    end
end

function s = fmtZero(x)
    if isfinite(x), s = sprintf('%.0f', x); else, s = 'never'; end
end

function tb = onsetDepart(t, y, tFa, tFb, tEnd, dirS, lev, holdT, minPts, minFit)
% Departure from the pre-maneuver TREND.  A line is fitted on the series'
% OWN samples in [tFa, tFb); the onset is the first instant after tFb at
% which dirS*(y - trend) exceeds lev (m/s, same level for every series)
% and stays above for holdT on native samples (>= minPts samples).  The
% instant is refined by linear interpolation.  NaN if the series is
% already beyond the level at tFb (start not separable from the trend).
    tb = NaN;  t = t(:);  y = y(:);
    f  = t >= tFa & t < tFb & isfinite(y);
    if nnz(f) < max(minFit, 3), return; end
    p  = [ones(nnz(f),1) t(f) - tFb] \ y(f);
    s  = t >= tFb & t <= tEnd & isfinite(y);
    ts = t(s);  d = dirS * (y(s) - (p(1) + p(2)*(ts - tFb)));
    n  = numel(ts);
    for i = 1:n
        if d(i) <= lev, continue; end
        jEnd = find(ts <= ts(i) + holdT, 1, 'last');
        jEnd = max(jEnd, min(i + minPts - 1, n));
        if jEnd >= n && ts(n) < ts(i) + holdT, return; end
        if all(d(i:jEnd) > lev)
            if i > 1 && d(i-1) <= lev
                tb = ts(i-1) + (lev - d(i-1)) * (ts(i) - ts(i-1)) / (d(i) - d(i-1));
            end
            return;
        end
    end
end
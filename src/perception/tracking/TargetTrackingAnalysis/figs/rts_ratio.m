%% ====================================================================
%  RTS gain matrix - confronto tra log con diverso Q
%
%  Popola "logs" e "qVals" qui sotto con i tuoi dati gia' caricati in
%  workspace (uno struct per log, come la variabile "log" usata finora),
%  poi esegui lo script.
% ====================================================================

%% ==================== LOG DA CONFRONTARE ====================
logs  = {log};        % <-- sostituisci con i tuoi log gia' caricati
qVals = [2000];        % <-- valore di Q corrispondente a ciascun log (stesso ordine)

%% ==================== parametri ====================
sl          = 1;
fldV        = 'opponents__smooth_ratio_v';   % rapporto P(k|N) / P(k|k) su vx
fldA        = 'opponents__smooth_ratio_a';   % rapporto P(k|N) / P(k|k) su ax
nLag        = 15;
gapMax      = 0.30;
useStamp    = false;

lagToCompare = 3;                  % lag usato per il confronto tra log (0 = piu' recente)
wTrend       = 1.00;               % s: finestra della media mobile
plotRaw      = true;               % true = mostra il grezzo tratteggiato sotto il trend
showMeanLine = true;                % true = linea orizzontale con la media scalare per log

%% ==================== estrazione per ciascun log ====================
nLogs = numel(logs);
assert(numel(qVals) == nLogs, 'qVals deve avere lo stesso numero di elementi di logs.');

SSv = cell(nLogs,1);  SSa = cell(nLogs,1);  dLagUsed = nan(nLogs,1);

for k = 1:nLogs
    [ssv, ssa, dlk] = extractGainAtLag(logs{k}, sl, fldV, fldA, nLag, useStamp, lagToCompare);
    SSv{k} = ssv;  SSa{k} = ssa;  dLagUsed(k) = dlk;
end

%% ==================== plot ====================
cm = turbo(nLogs);
[~, order] = sort(qVals);          % ordina la legenda per Q crescente

figure('Color','w');
axV = subplot(2,1,1); hold on; grid on;
for k = order
    plotTrend(SSv{k}, cm(k,:), gapMax, wTrend, plotRaw, showMeanLine, ...
              sprintf('Q = %.3g', qVals(k)));
end
ylabel(fldV, 'Interpreter', 'none');
legend('Location', 'eastoutside');

axA = subplot(2,1,2); hold on; grid on;
for k = order
    plotTrend(SSa{k}, cm(k,:), gapMax, wTrend, plotRaw, showMeanLine, ...
              sprintf('Q = %.3g', qVals(k)));
end
xlabel('time [s]'); ylabel(fldA, 'Interpreter', 'none');
legend('Location', 'eastoutside');

sgtitle('RTS gain matrix');
linkaxes([axV axA], 'x');


%% ==================== FUNZIONI LOCALI ====================
function [SSv, SSa, dLagOut] = extractGainAtLag(log, sl, fldV, fldA, nLag, useStamp, lagWanted)
% Estrae le serie [t, valore] di gain_v e gain_a per un singolo lag
% (0-based, lagWanted) da un log con la struttura opponents_history.
    oh = log.perception__opponents_history;
    T  = double(oh.('timestamp[]__tot'));
    nv = double(oh.opponents__steps(:, sl));

    st = double(log.perception__opponents.stamp__tot);  st(st==0) = NaN;
    t0 = min(st, [], 'omitnan');

    Gv = maskByStamps(squeeze(double(oh.(fldV)(:, sl, :))), nv);
    Ga = maskByStamps(squeeze(double(oh.(fldA)(:, sl, :))), nv);
    T  = maskByStamps(T, nv);

    Gv = fliplr(Gv);  Ga = fliplr(Ga);  T = fliplr(T);
    if useStamp && numel(st) == size(T,1), T(:,end) = st; end
    T = T - t0;

    tnow = nan(size(T,1),1);
    for r = 1:size(T,1)
        li = find(isfinite(T(r,:)), 1, 'last');
        if ~isempty(li), tnow(r) = T(r,li); end
    end

    N    = size(T,2);
    nLag = min(nLag, N);
    j    = lagWanted + 1;
    if j < 1 || j > nLag
        error('lagWanted=%d fuori range (0..%d) per questo log.', lagWanted, nLag-1);
    end
    col = N - (0:nLag-1);
    c   = col(j);

    lr = tnow - T(:,c);
    ok = nv >= j & isfinite(T(:,c)) & isfinite(lr) & lr >= 0;
    if nnz(ok) > 5
        lm = median(lr(ok));
        ok = ok & abs(lr - lm) < max(0.05, 0.5*lm);
    end
    dLagOut = NaN;
    if any(ok), dLagOut = median(lr(ok)) * 1e3; end

    ok = ok & isfinite(tnow);
    [tt, is] = sort(T(ok,c));
    yv = Gv(ok,c);  yv = yv(is);
    ya = Ga(ok,c);  ya = ya(is);
    [tt, iu] = unique(tt);  yv = yv(iu);  ya = ya(iu);
    SSv = [tt yv];
    SSa = [tt ya];
end

function X = maskByStamps(X, nv)
    for r = 1:size(X,1)
        if nv(r) >= 0 && nv(r) < size(X,2)
            X(r, nv(r)+1:end) = NaN;
        end
    end
end

function plotTrend(S, cc, gapMax, wTrend, plotRaw, showMeanLine, dispName)
    if isempty(S), return; end
    x = S(:,1);  y = S(:,2);

    if plotRaw
        xr = x; yr = y;
        br = find(diff(xr) > gapMax);
        for q = numel(br):-1:1
            xr = [xr(1:br(q)); NaN; xr(br(q)+1:end)];
            yr = [yr(1:br(q)); NaN; yr(br(q)+1:end)];
        end
        plot(xr, yr, ':', 'Color', [cc 0.30], 'LineWidth', 0.7, 'HandleVisibility', 'off');
    end

    yTrend = movmean(y, wTrend, 'SamplePoints', x);
    xp = x; yp = yTrend;
    br = find(diff(xp) > gapMax);
    for q = numel(br):-1:1
        xp = [xp(1:br(q)); NaN; xp(br(q)+1:end)];
        yp = [yp(1:br(q)); NaN; yp(br(q)+1:end)];
    end
    plot(xp, yp, '-', 'Color', cc, 'LineWidth', 2.0, 'DisplayName', dispName);

    if showMeanLine
        mv = mean(y, 'omitnan');
        yline(mv, ':', 'Color', cc, 'LineWidth', 1.0, 'HandleVisibility', 'off');
    end
end
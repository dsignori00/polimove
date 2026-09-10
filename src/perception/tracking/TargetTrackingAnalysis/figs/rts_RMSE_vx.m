%% RMSE_FIXED_LAG_VX
% RMSE tra vx stimata (perception__opponents_history) e ground truth, a un
% lag FISSO (~300 ms), per 1-3 log alla volta (log, log_2, log_3).
%
% Stesse convenzioni dello script di riferimento validato (per-lag
% analysis con nLag=15):
%   - fliplr(V), fliplr(T): colonna END del buffer grezzo = "adesso" (lag 0)
%   - riferimento per l'eta' del lag (dLag) = campione a LAG 1 (col(2)),
%     non lag 0 (nota: lag 0 uscira' con eta' leggermente NEGATIVA, e' atteso)
%   - popolazione RMSE = "common": campione valido su TUTTI gli nLag lag
%     testati, cosi' il confronto fra lag e' su basi comparabili
%
% SCRIPT (non function): lancialo da command window dopo aver caricato
% log/log_2/log_3/gt/compare/compare2 come fai di solito.

%% ==================== PARAMETRI ====================
sl            = 1;        % slot opponent
fld           = 'opponents__vx';
nLag          = 15;        % numero di lag su cui si costruisce la popolazione comune
useStamp      = false;     % true = lag 0 al publication timestamp (come nello script di rif.)
target_lag_ms = 300;       % lag desiderato in ms

%% ==================== ELENCO LOG DA PROCESSARE ====================
logs = {}; names = {}; shifts = [];

logs{end+1} = log;
if exist('name1','var'), names{end+1} = name1; else, names{end+1} = 'log'; end
shifts(end+1) = 0;

if exist('compare','var') && compare && exist('log_2','var')
    logs{end+1} = log_2;
    if exist('name2','var'), names{end+1} = name2; else, names{end+1} = 'log_2'; end
    shifts(end+1) = double(log_2.time_offset_nsec - log.time_offset_nsec) * 1e-9;
end

if exist('compare2','var') && compare2 && exist('log_3','var')
    logs{end+1} = log_3;
    if exist('name3','var'), names{end+1} = name3; else, names{end+1} = 'log_3'; end
    shifts(end+1) = double(log_3.time_offset_nsec - log.time_offset_nsec) * 1e-9;
end

nLogs = numel(logs);

%% ==================== GROUND TRUTH (comune, riferito al log primario) ====================
if ~exist('gt','var') || ~isfield(gt, 'vx')
    error('Variabile "gt" con campo vx non trovata nel workspace.');
end

rawOpp = [];
if isfield(log, 'perception__opponents')
    rawOpp = log.perception__opponents;
end
t0 = NaN;
if ~isempty(rawOpp) && isfield(rawOpp, 'stamp__tot')
    t0 = min(double(rawOpp.stamp__tot), [], 'omitnan');
end

gtT = double(gt.stamp) - t0;
gtV = double(gt.vx);
g   = isfinite(gtT) & isfinite(gtV);
[gtTu, iu] = unique(gtT(g));
gtVu = gtV(g);  gtVu = gtVu(iu);

%% ==================== LOOP SUI LOG ====================
results = struct('name', {}, 'RMSE', {}, 'dLag_ms', {}, 'N', {});

figure('Color','w');
tl = tiledlayout(nLogs, 1, 'TileSpacing','compact', 'Padding','compact');

for k = 1:nLogs
    Lk = logs{k};
    name_k = names{k};

    if ~isfield(Lk, 'perception__opponents_history')
        warning('rmse_fixed_lag_vx: "%s" non ha perception__opponents_history: log saltato.', name_k);
        continue;
    end

    oh = Lk.perception__opponents_history;
    T  = double(oh.('timestamp[]__tot'));
    nv = double(oh.opponents__steps(:, sl));

    st = double(Lk.perception__opponents.stamp__tot);  st(st==0) = NaN;
    t0k = min(st, [], 'omitnan');   % t0 locale al log (come nello script di riferimento)

    V = maskByStamps(squeeze(double(oh.(fld)(:, sl, :))), nv);
    T = maskByStamps(T, nv);

    V = fliplr(V);  T = fliplr(T);            % colonna end = "adesso"
    if useStamp && numel(st) == size(T,1), T(:,end) = st; end
    T = T - t0k + shifts(k);

    tnow = nan(size(T,1),1);
    for r = 1:size(T,1)
        li = find(isfinite(T(r,:)), 1, 'last');
        if ~isempty(li), tnow(r) = T(r,li); end
    end

    %% ---- maschere per lag ----
    N     = size(V,2);
    nLagK = min(nLag, N);
    col   = N - (0:nLagK-1);
    M     = false(size(V,1), nLagK);
    L     = nan(size(V,1), nLagK);

    % riferimento per l'eta' del lag = campione a LAG 1 (col(2)), non lag 0
    tref = T(:, col(2));

    for j = 1:nLagK
        c  = col(j);
        lr = tref - T(:,c);
        ok = nv >= j & isfinite(T(:,c)) & isfinite(V(:,c)) & V(:,c) ~= 0 ...
             & isfinite(lr) & isfinite(tref);
        if nnz(ok) > 5
            lm = median(lr(ok));
            ok = ok & abs(lr - lm) < max(0.05, 0.5*abs(lm));
        end
        M(:,j) = ok;  L(:,j) = lr;
    end

    dLag = nan(nLagK,1);
    for j = 1:nLagK
        if any(M(:,j)), dLag(j) = median(L(M(:,j), j)) * 1e3; end
    end

    %% ---- scelta lag piu' vicino a target_lag_ms ----
    [~, jSel] = min(abs(dLag - target_lag_ms));
    cSel = col(jSel);

    %% ---- popolazione comune (valida su tutti gli nLagK lag) ----
    common = all(M, 2) & isfinite(tnow);
    for j = 1:nLagK
        c = col(j);
        common = common & T(:,c) >= gtTu(1) & T(:,c) <= gtTu(end);
    end

    %% ---- RMSE a lag fisso, sulla popolazione comune ----
    tt = T(common, cSel);
    vv = V(common, cSel);
    gtInterp = interp1(gtTu, gtVu, tt, 'linear', NaN);

    e = vv - gtInterp;
    RMSE = sqrt(mean(e.^2, 'omitnan'));

    results(end+1).name    = name_k; %#ok<SAGROW>
    results(end).RMSE      = RMSE;
    results(end).dLag_ms   = dLag(jSel);
    results(end).N         = nnz(isfinite(e));

    fprintf('[%s] lag ~%.0f ms (target %.0f ms) - RMSE(vx,gt) = %.4f  (N = %d, popolazione comune su %d lag)\n', ...
            name_k, dLag(jSel), target_lag_ms, RMSE, results(end).N, nLagK);

    %% ---- plot ----
    nexttile;
    plot(tt, vv, '.', 'DisplayName', sprintf('%s: vx stimata (lag %.0f ms)', name_k, dLag(jSel))); hold on;
    plot(gtTu, gtVu, 'k-', 'LineWidth', 1.2, 'DisplayName', 'gt vx');
    grid on; box on;
    ylabel('v_x [m/s]');
    title(sprintf('%s - lag fisso %.0f ms - RMSE = %.4f', name_k, dLag(jSel), RMSE));
    legend('Location','best');
    if k == nLogs, xlabel('time [s]'); end
end

title(tl, sprintf('RMSE(vx,gt) a lag fisso ~%d ms - confronto %d log', target_lag_ms, numel(results)));

%% ==================== TABELLA RIASSUNTIVA ====================
Tres = struct2table(results);
disp(Tres);

%% ==================== FUNZIONI LOCALI ====================
function X = maskByStamps(X, nv)
% NaN sui campioni oltre il numero di step validi nv, per riga.
    for r = 1:size(X,1)
        if nv(r) >= 0 && nv(r) < size(X,2)
            X(r, nv(r)+1:end) = NaN;
        end
    end
end
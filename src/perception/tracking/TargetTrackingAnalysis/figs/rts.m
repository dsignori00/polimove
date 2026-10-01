% RLS - viewer interattivo del buffer storico OpponentHistory (slot 1).
% Plotta la velocita' longitudinale (vx): filtro smooth (buffer, scorre
% frame per frame), filtro normale, GT e misure quando disponibili.
%
% Se in TargetTrackingAnalysis sono attivi compare/compare2, plotta la
% history anche di log_2 / log_3 (un colore per log). I log privi di
% 'perception__opponents_history' vengono saltati con un avviso.
%
% SPAZIO/-> avanti | <- indietro | P play/pausa | +/- velocita' | HOME | q/ESC chiude.
%
% SCRIPT (non function): si lancia da main senza argomenti, legge 'log',
% 'log_2', 'log_3', 'gt', 'rad_clust', 'col', 'name1..3' dal workspace.

% ---------- parametri ----------
rls_slot    = 1;      % slot posizionale opponent
rls_period  = 0.1;    % s tra un frame e il successivo in play
rls_win     = 10;      % s, larghezza finestra scorrevole
rls_cosMin  = 0.15;   % |cos(aspect)| minimo per le misure radar
rls_rhoSign = 1;      % segno rho_dot
rls_radarVx = true;   % overlay misure radar rho_dot->vx

rls_field = 'vx';
rls_f = ['opponents__' rls_field];

% ---------- elenco log da plottare ----------
% ogni voce: variabile, nome, colore, shift temporale rispetto al log primario
rls_logs = {}; rls_names = {}; rls_cols = {}; rls_shifts = [];

rls_logs{end+1} = log;
if exist('name1','var'), rls_names{end+1} = name1; else, rls_names{end+1} = 'log'; end
if exist('col','var') && isfield(col,'tt'), rls_cols{end+1} = col.tt; else, rls_cols{end+1} = [0.10 0.40 0.85]; end
rls_shifts(end+1) = 0;

if exist('compare','var') && compare && exist('log_2','var')
    rls_logs{end+1} = log_2;
    if exist('name2','var'), rls_names{end+1} = name2; else, rls_names{end+1} = 'log_2'; end
    if exist('col','var') && isfield(col,'tt2'), rls_cols{end+1} = col.tt2; else, rls_cols{end+1} = [0.85 0.33 0.10]; end
    rls_shifts(end+1) = double(log_2.time_offset_nsec - log.time_offset_nsec)*1e-9;
end

if exist('compare2','var') && compare2 && exist('log_3','var')
    rls_logs{end+1} = log_3;
    if exist('name3','var'), rls_names{end+1} = name3; else, rls_names{end+1} = 'log_3'; end
    if exist('col','var') && isfield(col,'tt3'), rls_cols{end+1} = col.tt3; else, rls_cols{end+1} = [0.93 0.69 0.13]; end
    rls_shifts(end+1) = double(log_3.time_offset_nsec - log.time_offset_nsec)*1e-9;
end

% ---------- riferimento temporale comune (dal log primario) ----------
rls_rawOpp = [];
if isfield(log, 'perception__opponents')
    rls_rawOpp = log.perception__opponents;
end
rls_t0 = NaN;
if ~isempty(rls_rawOpp) && isfield(rls_rawOpp, 'stamp__tot')
    rls_t0 = min(double(rls_rawOpp.stamp__tot), [], 'omitnan');
end

% ---------- history di ciascun log ----------
rls_hist = struct('V', {}, 'T', {}, 'n', {}, 'name', {}, 'col', {}, 'ids', {}, 'nv', {});
for rls_k = 1:numel(rls_logs)
    rls_Lk = rls_logs{rls_k};
    if ~isfield(rls_Lk, 'perception__opponents_history')
        warning('rls: "%s" non ha perception__opponents_history: log saltato.', rls_names{rls_k});
        continue;
    end
    rls_ohk = rls_Lk.perception__opponents_history;
    rls_idsk = rls_ohk.opponents__obs_id;
    rls_nk = size(rls_idsk, 1);
    rls_nvk = double(rls_ohk.opponents__steps(:, rls_slot));

    rls_Vk = squeeze(double(rls_ohk.(rls_f)(:, rls_slot, :)));
    rls_Tk = double(rls_ohk.('timestamp[]__tot'));
    for rls_r = 1:rls_nk
        if rls_nvk(rls_r) < size(rls_Vk,2) && rls_nvk(rls_r) >= 0
            rls_Vk(rls_r, rls_nvk(rls_r)+1:end) = NaN;
            rls_Tk(rls_r, rls_nvk(rls_r)+1:end) = NaN;
        end
    end
    rls_Vk = fliplr(rls_Vk);
    rls_Tk = fliplr(rls_Tk);
    rls_Vk(rls_Vk == 0) = NaN;

    % colonna "adesso" = stamp__tot del proprio log (fonte ufficiale)
    if isfield(rls_Lk, 'perception__opponents') && isfield(rls_Lk.perception__opponents, 'stamp__tot')
        rls_stk = double(rls_Lk.perception__opponents.stamp__tot);
        if numel(rls_stk) == rls_nk
            rls_Tk(:, end) = rls_stk;
        end
    end

    % offset comune + shift del log rispetto al primario
    if isnan(rls_t0), rls_t0 = min(rls_Tk(:), [], 'omitnan'); end
    rls_Tk = rls_Tk - rls_t0 + rls_shifts(rls_k);

    rls_pre = rls_Tk < 0;
    rls_Tk(rls_pre) = NaN;
    rls_Vk(rls_pre) = NaN;

    rls_hist(end+1).V = rls_Vk; %#ok<SAGROW>
    rls_hist(end).T = rls_Tk;
    rls_hist(end).n = rls_nk;
    rls_hist(end).name = rls_names{rls_k};
    rls_hist(end).col = rls_cols{rls_k};
    rls_hist(end).ids = rls_idsk(:, rls_slot);
    rls_hist(end).nv = rls_nvk;
end

if isempty(rls_hist)
    error('rls: nessun log con perception__opponents_history disponibile.');
end

% ---------- filtro normale (log primario) ----------
rls_rawT = []; rls_rawV = [];
if ~isempty(rls_rawOpp) && isfield(rls_rawOpp, rls_f) && isfield(rls_rawOpp, 'stamp__tot')
    rls_rawV = double(rls_rawOpp.(rls_f)(:, rls_slot));
    rls_rawV(rls_rawV == 0) = NaN;
    rls_rawT = double(rls_rawOpp.stamp__tot) - rls_t0;
end

% ---------- GT ----------
rls_gtT = []; rls_gtV = [];
if exist('gt','var') && isfield(gt, 'vx')
    rls_gtT = double(gt.stamp) - rls_t0;
    rls_gtV = double(gt.vx);
end

% ---------- misure radar rho_dot -> vx ----------
rls_vxMeasV = []; rls_vxMeasTs = []; rls_vxMeasTa = [];
if rls_radarVx && exist('rad_clust','var') && ...
        isfield(log, 'estimation') && all(isfield(log.estimation, {'stamp__tot','vx'}))
    rls_s = rad_clust;
    rls_rd = double(rls_s.rho_dot(:,1));  rls_rd(rls_rd==0) = NaN;
    rls_xr = double(rls_s.x_rel(:,1));    rls_xr(rls_xr==0) = NaN;
    rls_yr = double(rls_s.y_rel(:,1));    rls_yr(rls_yr==0) = NaN;
    rls_yawr = double(rls_s.yaw_rel(:,1));
    if max(abs(rls_yawr), [], 'omitnan') > 2*pi
        rls_yawr = deg2rad(rls_yawr);
    end
    rls_yawr = wrapToPi(rls_yawr);

    rls_beta = atan2(rls_yr, rls_xr);
    rls_aspect = wrapToPi(rls_yawr - rls_beta);
    rls_c = cos(rls_aspect);
    rls_c(abs(rls_c) < rls_cosMin) = NaN;

    rls_teRaw = double(log.estimation.stamp__tot(:));
    rls_egoVxAll = double(log.estimation.vx(:));
    rls_egoValid = isfinite(rls_teRaw) & isfinite(rls_egoVxAll);

    % Keep timestamps aligned with the first detection selected above.
    % sens_stamp can have one column per detection, so flattening it with
    % (:) produces more samples than rls_rd, rls_beta, and rls_c.
    rls_sensStamp = double(rls_s.sens_stamp(:,1)); rls_sensStamp(rls_sensStamp==0) = NaN;
    rls_arrStamp = double(rls_s.stamp(:,1));       rls_arrStamp(rls_arrStamp==0) = NaN;

    rls_vegoI = interp1(rls_teRaw(rls_egoValid), rls_egoVxAll(rls_egoValid), ...
        rls_sensStamp, 'linear', 'extrap');
    rls_vxAll = (rls_rhoSign * rls_rd + rls_vegoI .* cos(rls_beta)) ./ rls_c;

    rls_goodMeas = isfinite(rls_vxAll) & isfinite(rls_sensStamp) & isfinite(rls_arrStamp);
    rls_vxMeasV = rls_vxAll(rls_goodMeas);
    rls_vxMeasTs = rls_sensStamp(rls_goodMeas) - rls_t0;
    rls_vxMeasTa = rls_arrStamp(rls_goodMeas) - rls_t0;
end

% ---------- figura ----------
rls_fig = figure('Color', 'w', 'Name', 'rls - history viewer', 'NumberTitle', 'off', ...
    'KeyPressFcn', @rls_key, 'CloseRequestFcn', @rls_stop);
rls_ax = axes(rls_fig);
hold(rls_ax, 'on'); grid(rls_ax, 'on');

if ~isempty(rls_gtT)
    plot(rls_ax, rls_gtT, rls_gtV, 'k-', 'DisplayName', 'GT');
end
if ~isempty(rls_rawT)
    plot(rls_ax, rls_rawT, rls_rawV, '-', 'Color', [0.90 0.55 0.10], ...
        'DisplayName', 'filtro normale');
end
if ~isempty(rls_vxMeasV)
    plot(rls_ax, rls_vxMeasTs, rls_vxMeasV, 'o', 'Color', [0.10 0.60 0.55], ...
        'MarkerSize', 4, 'LineStyle', 'none', 'DisplayName', 'radar (sens stamp)');
    plot(rls_ax, rls_vxMeasTa, rls_vxMeasV, 'x', 'Color', [0.10 0.60 0.55], ...
        'MarkerSize', 5, 'LineStyle', 'none', 'DisplayName', 'radar (filter stamp)');
end

rls_h = gobjects(1, numel(rls_hist));
for rls_k = 1:numel(rls_hist)
    rls_h(rls_k) = plot(rls_ax, nan, nan, 'o-', 'Color', rls_hist(rls_k).col, ...
        'MarkerFaceColor', rls_hist(rls_k).col, 'MarkerSize', 3, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('history %s', rls_hist(rls_k).name));
end

xlabel(rls_ax, 'tempo relativo [s]');
ylabel(rls_ax, 'vx [m/s]');
legend(rls_ax, 'Location', 'best');

% ---------- stato ----------
rls_startI = find(rls_hist(1).nv > 0, 1);
if isempty(rls_startI), rls_startI = 1; end

rls_st.hist = rls_hist;
rls_st.n = max([rls_hist.n]);
rls_st.i = rls_startI;
rls_st.startI = rls_startI;
rls_st.h = rls_h;
rls_st.ax = rls_ax;
rls_st.win = rls_win;
rls_st.slot = rls_slot;
rls_st.playing = false;
rls_st.period = rls_period;
rls_st.timer = timer('ExecutionMode', 'fixedRate', 'Period', rls_period, ...
    'TimerFcn', @(~,~) rls_draw_tick(rls_fig));
guidata(rls_fig, rls_st);
rls_draw(rls_fig);

clear rls_k rls_r rls_Lk rls_ohk rls_idsk rls_nk rls_nvk rls_Vk rls_Tk rls_stk rls_pre
clear rls_rawT rls_rawV rls_gtT rls_gtV rls_vxMeasV rls_vxMeasTs rls_vxMeasTa rls_st
clear rls_s rls_rd rls_xr rls_yr rls_yawr rls_beta rls_aspect rls_c rls_teRaw
clear rls_egoVxAll rls_egoValid rls_sensStamp rls_arrStamp rls_vegoI rls_vxAll rls_goodMeas
clear rls_logs rls_names rls_cols rls_shifts rls_f rls_field rls_hist

% ---------- callback ----------
function rls_draw_tick(fig)
if ~isvalid(fig), return; end
s = guidata(fig);
if s.i < s.n
    s.i = s.i + 1;
    guidata(fig, s);
    rls_draw(fig);
else
    stop(s.timer);
    s.playing = false;
    guidata(fig, s);
end
end

function rls_key(src, evt)
s = guidata(src);
switch evt.Key
    case {'space', 'rightarrow'}
        if s.playing, stop(s.timer); s.playing = false; end
        s.i = min(s.i + 1, s.n);
    case 'leftarrow'
        if s.playing, stop(s.timer); s.playing = false; end
        s.i = max(s.i - 1, 1);
    case 'home'
        if s.playing, stop(s.timer); s.playing = false; end
        s.i = s.startI;
    case 'p'
        if s.playing
            stop(s.timer); s.playing = false;
        else
            start(s.timer); s.playing = true;
        end
    case {'add', 'equal'}
        s.period = max(0.005, s.period / 1.5); s.timer.Period = s.period;
    case {'subtract', 'hyphen'}
        s.period = min(2, s.period * 1.5); s.timer.Period = s.period;
    case {'q', 'escape'}
        rls_stop(src);
        return;
    otherwise
        return;
end
guidata(src, s);
rls_draw(src);
end

function rls_stop(fig, ~)
try
    s = guidata(fig);
    if isfield(s, 'timer') && isvalid(s.timer)
        stop(s.timer);
        delete(s.timer);
    end
catch
end
delete(fig);
end

function rls_draw(fig)
s = guidata(fig);
curT = NaN;
for k = 1:numel(s.hist)
    ik = min(s.i, s.hist(k).n);
    x = s.hist(k).T(ik, :);
    y = s.hist(k).V(ik, :);
    set(s.h(k), 'XData', x, 'YData', y);
    lastValid = find(isfinite(x), 1, 'last');
    if k == 1 && ~isempty(lastValid), curT = x(lastValid); end
end
if isfinite(curT)
    xlim(s.ax, [curT - s.win, curT + 0.05 * s.win]);
end
if s.playing, status = 'PLAY'; else, status = 'pausa'; end
title(s.ax, sprintf('frame %d/%d | slot#%d obs id=%d | %s', ...
    s.i, s.n, s.slot, s.hist(1).ids(min(s.i, s.hist(1).n)), status));
drawnow limitrate;
end

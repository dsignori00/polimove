% RTS - viewer interattivo del buffer storico OpponentHistory (slot 1).
% ax: filtro smooth (buffer, scorre frame per frame), filtro normale, GT.
% Con compare/compare2 attivi plotta anche log_2 / log_3;
% i log senza 'perception__opponents_history' vengono saltati.
%
% NB rispetto alla versione su vx: qui non viene calcolata la misura radar,
% perche' rho_dot da' velocita' radiale e la conversione in accelerazione
% richiederebbe una derivata numerica separata, non la stessa geometria
% aspect/beta usata per vx. Se serve, va aggiunta a parte.
%
% I log secondari NON sono indicizzati per numero di frame ma agganciati
% al tempo del log primario (riga con "adesso" piu' vicino), cosi' restano
% allineati anche con rate diversi o messaggi mancanti.
%
% SPAZIO/-> avanti | <- indietro | P play/pausa | +/- velocita' | HOME | q/ESC chiude.
% Script: si lancia da main senza argomenti.

% ---------- parametri ----------
rts_slot    = 1;
rts_period  = 0.1;    % s tra frame in play
rts_win     = 3;      % s, finestra scorrevole

rts_field = 'ax';
rts_f = ['opponents__' rts_field];

% ---------- log da plottare: variabile, nome, colore, shift ----------
rts_logs = {}; rts_names = {}; rts_cols = {}; rts_shifts = [];

rts_logs{end+1} = log;
if exist('name1','var'), rts_names{end+1} = name1; else, rts_names{end+1} = 'log'; end
if exist('col','var') && isfield(col,'tt'), rts_cols{end+1} = col.tt; else, rts_cols{end+1} = [0.10 0.40 0.85]; end
rts_shifts(end+1) = 0;

if exist('compare','var') && compare && exist('log_2','var')
    rts_logs{end+1} = log_2;
    if exist('name2','var'), rts_names{end+1} = name2; else, rts_names{end+1} = 'log_2'; end
    if exist('col','var') && isfield(col,'tt2'), rts_cols{end+1} = col.tt2; else, rts_cols{end+1} = [0.85 0.33 0.10]; end
    rts_shifts(end+1) = double(log_2.time_offset_nsec - log.time_offset_nsec)*1e-9;
end

if exist('compare2','var') && compare2 && exist('log_3','var')
    rts_logs{end+1} = log_3;
    if exist('name3','var'), rts_names{end+1} = name3; else, rts_names{end+1} = 'log_3'; end
    if exist('col','var') && isfield(col,'tt3'), rts_cols{end+1} = col.tt3; else, rts_cols{end+1} = [0.93 0.69 0.13]; end
    rts_shifts(end+1) = double(log_3.time_offset_nsec - log.time_offset_nsec)*1e-9;
end

% ---------- riferimento temporale (log primario) ----------
rts_rawOpp = [];
if isfield(log, 'perception__opponents')
    rts_rawOpp = log.perception__opponents;
end
rts_t0 = NaN;
if ~isempty(rts_rawOpp) && isfield(rts_rawOpp, 'stamp__tot')
    rts_st0 = double(rts_rawOpp.stamp__tot);
    rts_st0(rts_st0 == 0) = NaN;          % zeri = campi non popolati
    rts_t0 = min(rts_st0, [], 'omitnan');
end

% ---------- history di ciascun log ----------
rts_hist = struct('V', {}, 'T', {}, 'n', {}, 'name', {}, 'col', {}, 'ids', {}, ...
                  'nv', {}, 'tnow', {});
for rts_k = 1:numel(rts_logs)
    rts_Lk = rts_logs{rts_k};
    if ~isfield(rts_Lk, 'perception__opponents_history')
        warning('rts: "%s" non ha perception__opponents_history: saltato.', rts_names{rts_k});
        continue;
    end
    rts_ohk = rts_Lk.perception__opponents_history;
    if ~isfield(rts_ohk, rts_f)
        warning('rts: "%s" non ha il campo %s nel buffer: saltato.', rts_names{rts_k}, rts_f);
        continue;
    end
    rts_idsk = rts_ohk.opponents__obs_id;
    rts_nk = size(rts_idsk, 1);
    rts_nvk = double(rts_ohk.opponents__steps(:, rts_slot));

    rts_Vk = squeeze(double(rts_ohk.(rts_f)(:, rts_slot, :)));
    rts_Tk = double(rts_ohk.('timestamp[]__tot'));
    for rts_r = 1:rts_nk
        if rts_nvk(rts_r) < size(rts_Vk,2) && rts_nvk(rts_r) >= 0
            rts_Vk(rts_r, rts_nvk(rts_r)+1:end) = NaN;
            rts_Tk(rts_r, rts_nvk(rts_r)+1:end) = NaN;
        end
    end
    rts_Vk = fliplr(rts_Vk);
    rts_Tk = fliplr(rts_Tk);
    % NB: niente filtro V==0 -> per l'accelerazione zero e' un valore
    % legittimo (target non sta accelerando), a differenza di vx.

    % colonna "adesso" = stamp__tot del proprio log
    if isfield(rts_Lk, 'perception__opponents') && isfield(rts_Lk.perception__opponents, 'stamp__tot')
        rts_stk = double(rts_Lk.perception__opponents.stamp__tot);
        if numel(rts_stk) == rts_nk
            rts_Tk(:, end) = rts_stk;
        end
    end

    if isnan(rts_t0), rts_t0 = min(rts_Tk(:), [], 'omitnan'); end
    rts_Tk = rts_Tk - rts_t0 + rts_shifts(rts_k);

    rts_pre = rts_Tk < 0;   % storia precedente al primo messaggio: scartata
    rts_Tk(rts_pre) = NaN;
    rts_Vk(rts_pre) = NaN;

    rts_hist(end+1).V = rts_Vk; %#ok<SAGROW>
    rts_hist(end).T = rts_Tk;
    rts_hist(end).n = rts_nk;
    rts_hist(end).name = rts_names{rts_k};
    rts_hist(end).col = rts_cols{rts_k};
    rts_hist(end).ids = rts_idsk(:, rts_slot);
    rts_hist(end).nv = rts_nvk;
end

if isempty(rts_hist)
    error('rts: nessun log con perception__opponents_history e campo %s disponibile.', rts_f);
end

% ---------- tempo "adesso" di ogni riga (per l'aggancio temporale) ----------
for rts_k = 1:numel(rts_hist)
    rts_Tk = rts_hist(rts_k).T;
    rts_tnow = nan(size(rts_Tk, 1), 1);
    for rts_r = 1:size(rts_Tk, 1)
        rts_li = find(isfinite(rts_Tk(rts_r, :)), 1, 'last');
        if ~isempty(rts_li)
            rts_tnow(rts_r) = rts_Tk(rts_r, rts_li);
        end
    end
    rts_hist(rts_k).tnow = rts_tnow;
end

% ---------- filtro normale (log primario) ----------
rts_rawT = []; rts_rawV = [];
if ~isempty(rts_rawOpp) && isfield(rts_rawOpp, rts_f) && isfield(rts_rawOpp, 'stamp__tot')
    rts_rawV = double(rts_rawOpp.(rts_f)(:, rts_slot));
    rts_rawT = double(rts_rawOpp.stamp__tot) - rts_t0;
end

% ---------- GT ----------
rts_gtT = []; rts_gtV = [];
if exist('gt','var') && isfield(gt, rts_field)
    rts_gtT = double(gt.stamp) - rts_t0;
    rts_gtV = double(gt.(rts_field));
elseif exist('gt','var') && isfield(gt, 'vx')
    warning('rts: gt.%s non trovato, GT accelerazione non plottato (vedi derivata di gt.vx se serve).', rts_field);
end

% ---------- figura ----------
rts_fig = figure('Color', 'w', 'Name', 'rts - history viewer (ax)', 'NumberTitle', 'off', ...
    'KeyPressFcn', @rts_key, 'CloseRequestFcn', @rts_stop);
rts_ax = axes(rts_fig);
hold(rts_ax, 'on'); grid(rts_ax, 'on');

if ~isempty(rts_gtT)
    plot(rts_ax, rts_gtT, rts_gtV, 'k-', 'DisplayName', 'GT');
end
if ~isempty(rts_rawT)
    plot(rts_ax, rts_rawT, rts_rawV, '-', 'Color', [0.90 0.55 0.10], ...
        'DisplayName', 'filtro normale');
end

rts_h = gobjects(1, numel(rts_hist));
for rts_k = 1:numel(rts_hist)
    rts_h(rts_k) = plot(rts_ax, nan, nan, 'o-', 'Color', rts_hist(rts_k).col, ...
        'MarkerFaceColor', rts_hist(rts_k).col, 'MarkerSize', 3, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('history %s', rts_hist(rts_k).name));
end

xlabel(rts_ax, 'time [s]');
ylabel(rts_ax, 'a_x [m/s^2]');
rts_lgd = legend(rts_ax, 'Location', 'south', 'Orientation', 'horizontal', ...
    'AutoUpdate', 'off');
set(rts_lgd, 'Color', 'none', 'EdgeColor', [0.7 0.7 0.7], 'Box', 'on');

% ---------- stato ----------
rts_startI = find(rts_hist(1).nv > 0, 1);
if isempty(rts_startI), rts_startI = 1; end

rts_st.hist = rts_hist;
rts_st.n = rts_hist(1).n;      % la riproduzione segue il log primario
rts_st.i = rts_startI;
rts_st.startI = rts_startI;
rts_st.h = rts_h;
rts_st.ax = rts_ax;
rts_st.win = rts_win;
rts_st.slot = rts_slot;
rts_st.playing = false;
rts_st.period = rts_period;
rts_st.timer = timer('ExecutionMode', 'fixedRate', 'Period', rts_period, ...
    'TimerFcn', @(~,~) rts_draw_tick(rts_fig));
guidata(rts_fig, rts_st);
rts_draw(rts_fig);

clear rts_k rts_r rts_Lk rts_ohk rts_idsk rts_nk rts_nvk rts_Vk rts_Tk rts_stk rts_pre
clear rts_rawT rts_rawV rts_gtT rts_gtV rts_st
clear rts_logs rts_names rts_cols rts_shifts rts_f rts_field rts_hist
clear rts_st0 rts_tnow rts_li rts_lgd

% ---------- callback ----------
function rts_draw_tick(fig)
if ~isvalid(fig), return; end
s = guidata(fig);
if s.i < s.n
    s.i = s.i + 1;
    guidata(fig, s);
    rts_draw(fig);
else
    stop(s.timer);
    s.playing = false;
    guidata(fig, s);
end
end

function rts_key(src, evt)
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
        rts_stop(src);
        return;
    otherwise
        return;
end
guidata(src, s);
rts_draw(src);
end

function rts_stop(fig, ~)
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

function rts_draw(fig)
s = guidata(fig);

% log primario: indice = frame corrente, e' lui a comandare la riproduzione
i1 = min(s.i, s.hist(1).n);
curT = s.hist(1).tnow(i1);

for k = 1:numel(s.hist)
    if k == 1
        ik = i1;
    elseif isfinite(curT)
        % log secondari: riga con "adesso" piu' vicino al tempo del log 1
        d = abs(s.hist(k).tnow - curT);
        if all(isnan(d))
            ik = min(s.i, s.hist(k).n);
        else
            [~, ik] = min(d, [], 'omitnan');
        end
    else
        ik = min(s.i, s.hist(k).n);
    end
    set(s.h(k), 'XData', s.hist(k).T(ik, :), 'YData', s.hist(k).V(ik, :));
end

if isfinite(curT)
    xlim(s.ax, [curT - s.win, curT + 0.05 * s.win]);
end

if s.playing, status = 'PLAY'; else, status = 'pausa'; end
title(s.ax, sprintf('frame %d/%d | slot#%d obs id=%d | %s', ...
    s.i, s.n, s.slot, s.hist(1).ids(i1), status));
drawnow limitrate;
end
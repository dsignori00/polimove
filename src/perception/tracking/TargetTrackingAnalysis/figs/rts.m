% RTS - viewer interattivo del buffer storico OpponentHistory (slot 1).
% Nel pannello superiore plotta la velocita' longitudinale (vx); nel
% pannello inferiore l'accelerazione longitudinale (ax). Per entrambe mostra
% il filtro smooth (buffer, scorre frame per frame), il filtro normale e il
% GT quando disponibile. Le misure radar, se abilitate, sono mostrate solo
% nel pannello della velocita'.
%
% Se in TargetTrackingAnalysis sono attivi compare/compare2, plotta la
% history anche di log_2 / log_3 (un colore per log). I log secondari privi
% dei campi richiesti vengono saltati con un avviso.
%
% SPAZIO/-> avanti | <- indietro | P play/pausa | +/- velocita' | HOME | q/ESC chiude.
%
% SCRIPT (non function): si lancia da main senza argomenti, legge 'log',
% 'log_2', 'log_3', 'gt', 'rad_clust', 'col', 'name1..3' dal workspace.

% ---------- parametri ----------
rts_slot = 1;        % slot posizionale opponent
rts_period = 0.1;    % s tra un frame e il successivo in play
rts_cosMin = 0.15;   % |cos(aspect)| minimo per le misure radar
rts_rhoSign = 1;     % segno rho_dot
rts_causalLightening = 0.45; % 0 = colore history, 1 = bianco
rts_smoothGapMax = 0.30;     % s, interrompe la curva smooth tra segmenti lontani
if ~exist('rts_radarVx', 'var')
    rts_radarVx = false; % overlay misure radar rho_dot->vx
end

rts_vxField = 'opponents__vx';
rts_axField = 'opponents__ax';
rts_requiredHistoryFields = { ...
    'opponents__obs_id', 'opponents__steps', rts_vxField, ...
    rts_axField, 'timestamp[]__tot'};

% ---------- elenco log da plottare ----------
% Ogni voce: variabile, nome, colore, shift temporale rispetto al log primario.
rts_logs = {log};
if exist('name1', 'var')
    rts_names = {name1};
else
    rts_names = {'log'};
end
if exist('col', 'var') && isfield(col, 'tt')
    rts_cols = {col.tt};
else
    rts_cols = {[0.10 0.40 0.85]};
end
rts_shifts = 0;

if exist('compare', 'var') && compare && exist('log_2', 'var')
    rts_logs{end+1} = log_2;
    if exist('name2', 'var')
        rts_names{end+1} = name2;
    else
        rts_names{end+1} = 'log_2';
    end
    if exist('col', 'var') && isfield(col, 'tt2')
        rts_cols{end+1} = col.tt2;
    else
        rts_cols{end+1} = [0.85 0.33 0.10];
    end
    rts_shifts(end+1) = double(log_2.time_offset_nsec - log.time_offset_nsec)*1e-9;
end

if exist('compare2', 'var') && compare2 && exist('log_3', 'var')
    rts_logs{end+1} = log_3;
    if exist('name3', 'var')
        rts_names{end+1} = name3;
    else
        rts_names{end+1} = 'log_3';
    end
    if exist('col', 'var') && isfield(col, 'tt3')
        rts_cols{end+1} = col.tt3;
    else
        rts_cols{end+1} = [0.93 0.69 0.13];
    end
    rts_shifts(end+1) = double(log_3.time_offset_nsec - log.time_offset_nsec)*1e-9;
end

% ---------- riferimento temporale comune (dal log primario) ----------
rts_rawOpp = [];
if isfield(log, 'perception__opponents')
    rts_rawOpp = log.perception__opponents;
end
rts_t0 = NaN;
if ~isempty(rts_rawOpp) && isfield(rts_rawOpp, 'stamp__tot')
    rts_primaryStamps = double(rts_rawOpp.stamp__tot(:));
    rts_primaryStamps(rts_primaryStamps == 0) = NaN;
    rts_t0 = min(rts_primaryStamps, [], 'omitnan');
end

% ---------- history di ciascun log ----------
% Il campo 'now' sincronizza log con numero di snapshot o rate diversi.
rts_hist = struct('Vx', {}, 'Ax', {}, 'T', {}, 'smoothT', {}, ...
    'smoothVx', {}, 'smoothAx', {}, 'n', {}, 'name', {}, ...
    'col', {}, 'ids', {}, 'nv', {}, 'now', {});
for rts_k = 1:numel(rts_logs)
    rts_Lk = rts_logs{rts_k};
    if ~isfield(rts_Lk, 'perception__opponents_history')
        rts_problem = sprintf('"%s" non ha perception__opponents_history.', rts_names{rts_k});
        if rts_k == 1
            error('rts:PrimaryHistoryMissing', 'rts: %s Il log primario e'' necessario.', rts_problem);
        end
        warning('rts:SecondaryHistoryMissing', 'rts: %s Log saltato.', rts_problem);
        continue;
    end

    rts_ohk = rts_Lk.perception__opponents_history;
    rts_missingFields = rts_requiredHistoryFields(~isfield(rts_ohk, rts_requiredHistoryFields));
    if ~isempty(rts_missingFields)
        rts_problem = sprintf('"%s" non ha i campi history: %s.', ...
            rts_names{rts_k}, strjoin(rts_missingFields, ', '));
        if rts_k == 1
            error('rts:PrimaryHistoryInvalid', 'rts: %s', rts_problem);
        end
        warning('rts:SecondaryHistoryInvalid', 'rts: %s Log saltato.', rts_problem);
        continue;
    end

    rts_idsk = rts_ohk.opponents__obs_id;
    rts_steps = rts_ohk.opponents__steps;
    rts_vxData = rts_ohk.(rts_vxField);
    rts_axData = rts_ohk.(rts_axField);
    if any([size(rts_idsk, 2), size(rts_steps, 2), ...
            size(rts_vxData, 2), size(rts_axData, 2)] < rts_slot)
        rts_problem = sprintf('"%s" non contiene lo slot %d.', rts_names{rts_k}, rts_slot);
        if rts_k == 1
            error('rts:PrimarySlotMissing', 'rts: %s', rts_problem);
        end
        warning('rts:SecondarySlotMissing', 'rts: %s Log saltato.', rts_problem);
        continue;
    end

    rts_nk = size(rts_idsk, 1);
    rts_nvk = double(rts_steps(:, rts_slot));
    rts_Vxk = rts_history_matrix(rts_vxData, rts_slot);
    rts_Axk = rts_history_matrix(rts_axData, rts_slot);
    rts_Tk = double(rts_ohk.('timestamp[]__tot'));
    if size(rts_Tk, 1) ~= rts_nk || ...
            ~isequal(size(rts_Tk), size(rts_Vxk), size(rts_Axk))
        rts_problem = sprintf('"%s" ha dimensioni history incoerenti.', rts_names{rts_k});
        if rts_k == 1
            error('rts:PrimaryHistorySizeMismatch', 'rts: %s', rts_problem);
        end
        warning('rts:SecondaryHistorySizeMismatch', 'rts: %s Log saltato.', rts_problem);
        continue;
    end

    % L'ultimo elemento valido nel buffer originale e' il campione piu'
    % vecchio, quindi quello che ha ricevuto il maggior numero di update RTS.
    [rts_smoothTk, rts_smoothVxk, rts_smoothAxk] = ...
        rts_finalized_trace(rts_Tk, rts_Vxk, rts_Axk, rts_nvk, rts_smoothGapMax);

    rts_Vxk = rts_mask_history(rts_Vxk, rts_nvk);
    rts_Axk = rts_mask_history(rts_Axk, rts_nvk);
    rts_Tk = rts_mask_history(rts_Tk, rts_nvk);
    rts_Vxk = fliplr(rts_Vxk);
    rts_Axk = fliplr(rts_Axk);
    rts_Tk = fliplr(rts_Tk);
    rts_Vxk(rts_Vxk == 0) = NaN;

    % Colonna "adesso" = stamp__tot del proprio log.
    if isfield(rts_Lk, 'perception__opponents') && ...
            isfield(rts_Lk.perception__opponents, 'stamp__tot')
        rts_stk = double(rts_Lk.perception__opponents.stamp__tot(:));
        rts_stk(rts_stk == 0) = NaN;
        if numel(rts_stk) == rts_nk
            rts_Tk(:, end) = rts_stk;
        end
    end

    if isnan(rts_t0)
        rts_validTimes = rts_Tk(:);
        rts_validTimes(rts_validTimes == 0) = NaN;
        rts_t0 = min(rts_validTimes, [], 'omitnan');
    end
    if isnan(rts_t0)
        error('rts:TimeReferenceMissing', ...
            'rts: impossibile determinare un timestamp valido per il log primario.');
    end
    rts_Tk = rts_Tk - rts_t0 + rts_shifts(rts_k);
    rts_smoothTk = rts_smoothTk - rts_t0 + rts_shifts(rts_k);

    rts_pre = rts_Tk < 0;
    rts_Tk(rts_pre) = NaN;
    rts_Vxk(rts_pre) = NaN;
    rts_Axk(rts_pre) = NaN;
    rts_smoothPre = rts_smoothTk < 0;
    rts_smoothTk(rts_smoothPre) = NaN;
    rts_smoothVxk(rts_smoothPre) = NaN;
    rts_smoothAxk(rts_smoothPre) = NaN;

    rts_hist(end+1).Vx = rts_Vxk; %#ok<SAGROW>
    rts_hist(end).Ax = rts_Axk;
    rts_hist(end).T = rts_Tk;
    rts_hist(end).smoothT = rts_smoothTk;
    rts_hist(end).smoothVx = rts_smoothVxk;
    rts_hist(end).smoothAx = rts_smoothAxk;
    rts_hist(end).n = rts_nk;
    rts_hist(end).name = rts_names{rts_k};
    rts_hist(end).col = rts_cols{rts_k};
    rts_hist(end).ids = rts_idsk(:, rts_slot);
    rts_hist(end).nv = rts_nvk;
    rts_hist(end).now = rts_Tk(:, end);
end

% ---------- filtro normale ("history 0") per ciascun log ----------
rts_raw = struct('T', {}, 'Vx', {}, 'Ax', {}, 'name', {}, 'col', {});
for rts_k = 1:numel(rts_logs)
    rts_Lk = rts_logs{rts_k};
    if ~isfield(rts_Lk, 'perception__opponents')
        continue;
    end
    rts_oppk = rts_Lk.perception__opponents;
    rts_requiredRawFields = {'stamp__tot', rts_vxField, rts_axField};
    if ~all(isfield(rts_oppk, rts_requiredRawFields)) || ...
            size(rts_oppk.(rts_vxField), 2) < rts_slot || ...
            size(rts_oppk.(rts_axField), 2) < rts_slot
        continue;
    end

    rts_rawVx = double(rts_oppk.(rts_vxField)(:, rts_slot));
    rts_rawAx = double(rts_oppk.(rts_axField)(:, rts_slot));
    rts_rawVx(rts_rawVx == 0) = NaN;
    rts_rawT = double(rts_oppk.stamp__tot(:)) - rts_t0 + rts_shifts(rts_k);

    rts_raw(end+1).T = rts_rawT; %#ok<SAGROW>
    rts_raw(end).Vx = rts_rawVx;
    rts_raw(end).Ax = rts_rawAx;
    rts_raw(end).name = rts_names{rts_k};
    rts_raw(end).col = rts_cols{rts_k};
end

% ---------- GT ----------
rts_gtT = [];
rts_gtVx = [];
rts_gtAx = [];
if exist('gt', 'var') && isfield(gt, 'stamp')
    rts_gtT = double(gt.stamp(:)) - rts_t0;
    if isfield(gt, 'vx')
        rts_gtVx = double(gt.vx(:));
    end
    if isfield(gt, 'ax')
        rts_gtAx = double(gt.ax(:));
    end
end

% ---------- misure radar rho_dot -> vx (solo pannello superiore) ----------
rts_vxMeasV = [];
rts_vxMeasTs = [];
rts_vxMeasTa = [];
rts_radarFields = {'rho_dot', 'x_rel', 'y_rel', 'yaw_rel', 'sens_stamp', 'stamp'};
if rts_radarVx && exist('rad_clust', 'var') && ...
        all(isfield(rad_clust, rts_radarFields)) && ...
        isfield(log, 'estimation') && ...
        all(isfield(log.estimation, {'stamp__tot', 'vx'}))
    rts_rd = double(rad_clust.rho_dot(:, 1));
    rts_xr = double(rad_clust.x_rel(:, 1));
    rts_yr = double(rad_clust.y_rel(:, 1));
    rts_yawr = double(rad_clust.yaw_rel(:, 1));
    rts_rd(rts_rd == 0) = NaN;
    rts_xr(rts_xr == 0) = NaN;
    rts_yr(rts_yr == 0) = NaN;
    if max(abs(rts_yawr), [], 'omitnan') > 2*pi
        rts_yawr = deg2rad(rts_yawr);
    end
    rts_yawr = wrapToPi(rts_yawr);

    rts_beta = atan2(rts_yr, rts_xr);
    rts_aspect = wrapToPi(rts_yawr - rts_beta);
    rts_c = cos(rts_aspect);
    rts_c(abs(rts_c) < rts_cosMin) = NaN;

    rts_teRaw = double(log.estimation.stamp__tot(:));
    rts_egoVxAll = double(log.estimation.vx(:));
    rts_egoValid = isfinite(rts_teRaw) & isfinite(rts_egoVxAll);
    [rts_egoT, rts_uniqueI] = unique(rts_teRaw(rts_egoValid));
    rts_egoVx = rts_egoVxAll(rts_egoValid);
    rts_egoVx = rts_egoVx(rts_uniqueI);

    % Tutti i segnali radar si riferiscono alla prima detection selezionata.
    rts_sensStamp = double(rad_clust.sens_stamp(:, 1));
    rts_arrStamp = double(rad_clust.stamp(:, 1));
    rts_sensStamp(rts_sensStamp == 0) = NaN;
    rts_arrStamp(rts_arrStamp == 0) = NaN;

    if numel(rts_egoT) >= 2
        rts_vegoI = interp1(rts_egoT, rts_egoVx, rts_sensStamp, 'linear', 'extrap');
        rts_vxAll = (rts_rhoSign*rts_rd + rts_vegoI.*cos(rts_beta))./rts_c;
        rts_goodMeas = isfinite(rts_vxAll) & isfinite(rts_sensStamp) & isfinite(rts_arrStamp);
        rts_vxMeasV = rts_vxAll(rts_goodMeas);
        rts_vxMeasTs = rts_sensStamp(rts_goodMeas) - rts_t0;
        rts_vxMeasTa = rts_arrStamp(rts_goodMeas) - rts_t0;
    else
        warning('rts:RadarEgoSpeedUnavailable', ...
            'rts: almeno due campioni validi di velocita'' ego sono necessari per il radar.');
    end
end

% ---------- figura ----------
rts_fig = figure('Color', 'w', 'Name', 'RTS - history viewer', ...
    'NumberTitle', 'off', 'KeyPressFcn', @rts_key, ...
    'CloseRequestFcn', @rts_stop);
rts_layout = tiledlayout(rts_fig, 2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
rts_axVx = nexttile(rts_layout, 1);
rts_axAx = nexttile(rts_layout, 2);
hold(rts_axVx, 'on');
hold(rts_axAx, 'on');
grid(rts_axVx, 'on');
grid(rts_axAx, 'on');

if ~isempty(rts_gtVx)
    plot(rts_axVx, rts_gtT, rts_gtVx, 'k-', 'DisplayName', 'GT');
end
if ~isempty(rts_gtAx)
    plot(rts_axAx, rts_gtT, rts_gtAx, 'k-', 'DisplayName', 'GT');
end
for rts_k = 1:numel(rts_raw)
    rts_causalColor = (1 - rts_causalLightening)*rts_raw(rts_k).col + ...
        rts_causalLightening*[1 1 1];
    plot(rts_axVx, rts_raw(rts_k).T, rts_raw(rts_k).Vx, '--', ...
        'Color', rts_causalColor, ...
        'DisplayName', sprintf('causal %s', rts_raw(rts_k).name));
    plot(rts_axAx, rts_raw(rts_k).T, rts_raw(rts_k).Ax, '--', ...
        'Color', rts_causalColor, ...
        'DisplayName', sprintf('causal %s', rts_raw(rts_k).name));
end
if ~isempty(rts_vxMeasV)
    plot(rts_axVx, rts_vxMeasTs, rts_vxMeasV, 'o', ...
        'Color', [0.10 0.60 0.55], 'MarkerSize', 4, 'LineStyle', 'none', ...
        'DisplayName', 'radar (sens stamp)');
    plot(rts_axVx, rts_vxMeasTa, rts_vxMeasV, 'x', ...
        'Color', [0.10 0.60 0.55], 'MarkerSize', 5, 'LineStyle', 'none', ...
        'DisplayName', 'radar (filter stamp)');
end

rts_hVx = gobjects(1, numel(rts_hist));
rts_hAx = gobjects(1, numel(rts_hist));
for rts_k = 1:numel(rts_hist)
    plot(rts_axVx, rts_hist(rts_k).smoothT, rts_hist(rts_k).smoothVx, '-', ...
        'Color', rts_hist(rts_k).col, 'LineWidth', 1.5, ...
        'HandleVisibility', 'off');
    plot(rts_axAx, rts_hist(rts_k).smoothT, rts_hist(rts_k).smoothAx, '-', ...
        'Color', rts_hist(rts_k).col, 'LineWidth', 1.5, ...
        'HandleVisibility', 'off');
    rts_hVx(rts_k) = plot(rts_axVx, nan, nan, 'o-', ...
        'Color', rts_hist(rts_k).col, 'MarkerFaceColor', rts_hist(rts_k).col, ...
        'MarkerSize', 3, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('history %s', rts_hist(rts_k).name));
    rts_hAx(rts_k) = plot(rts_axAx, nan, nan, 'o-', ...
        'Color', rts_hist(rts_k).col, 'MarkerFaceColor', rts_hist(rts_k).col, ...
        'MarkerSize', 3, 'LineWidth', 1.2, ...
        'DisplayName', sprintf('history %s', rts_hist(rts_k).name));
end

ylabel(rts_axVx, 'vx [m/s]');
ylabel(rts_axAx, 'ax [m/s2]');
xlabel(rts_axAx, 'timestamp [s]');
legend(rts_axVx, 'Location', 'northwest');
legend(rts_axAx, 'Location', 'northwest');
linkaxes([rts_axVx rts_axAx], 'x');

% Registra i due pannelli nel linking globale di TargetTrackingAnalysis.
if ~exist('ax', 'var') || isempty(ax)
    ax = gobjects(0);
end
if ~exist('f', 'var') || isempty(f)
    f = numel(ax) + 1;
end
ax(f) = rts_axVx;
f = f + 1;
ax(f) = rts_axAx;
f = f + 1;

% ---------- stato ----------
rts_dataStartI = find(rts_hist(1).nv > 0, 1);
if isempty(rts_dataStartI)
    rts_dataStartI = 1;
end

rts_st.hist = rts_hist;
rts_st.n = rts_hist(1).n;
rts_st.i = rts_dataStartI;
rts_st.dataStartI = rts_dataStartI;
rts_st.hVx = rts_hVx;
rts_st.hAx = rts_hAx;
rts_st.axVx = rts_axVx;
rts_st.axAx = rts_axAx;
rts_st.slot = rts_slot;
rts_st.playing = false;
rts_st.period = rts_period;
rts_st.timer = timer('ExecutionMode', 'fixedRate', 'Period', rts_period, ...
    'TimerFcn', @(~, ~) rts_draw_tick(rts_fig));
guidata(rts_fig, rts_st);
rts_draw(rts_fig);

clearvars('-regexp', '^rts_');

% ---------- callback ----------
function rts_draw_tick(fig)
if ~isvalid(fig)
    return;
end
s = guidata(fig);
[firstFrame, lastFrame] = rts_frame_bounds(s);
if s.i < firstFrame || s.i > lastFrame
    s.i = firstFrame;
end
if s.i < lastFrame
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
[firstFrame, lastFrame] = rts_frame_bounds(s);
switch evt.Key
    case {'space', 'rightarrow'}
        if s.playing
            stop(s.timer);
            s.playing = false;
        end
        s.i = min(max(s.i + 1, firstFrame), lastFrame);
    case 'leftarrow'
        if s.playing
            stop(s.timer);
            s.playing = false;
        end
        s.i = max(min(s.i - 1, lastFrame), firstFrame);
    case 'home'
        if s.playing
            stop(s.timer);
            s.playing = false;
        end
        s.i = firstFrame;
    case 'p'
        if s.playing
            stop(s.timer);
            s.playing = false;
        else
            if s.i < firstFrame || s.i >= lastFrame
                s.i = firstFrame;
            end
            start(s.timer);
            s.playing = true;
        end
    case {'add', 'equal'}
        s.period = max(0.005, s.period/1.5);
        s.timer.Period = s.period;
    case {'subtract', 'hyphen'}
        s.period = min(2, s.period*1.5);
        s.timer.Period = s.period;
    case {'q', 'escape'}
        rts_stop(src, []);
        return;
    otherwise
        return;
end
guidata(src, s);
rts_draw(src);
end

function rts_stop(fig, ~)
if ~isvalid(fig)
    return;
end
s = guidata(fig);
if isstruct(s) && isfield(s, 'timer') && isvalid(s.timer)
    stop(s.timer);
    delete(s.timer);
end
delete(fig);
end

function rts_draw(fig)
s = guidata(fig);

% Il log primario comanda il frame e il riferimento temporale.
primaryIndex = min(s.i, s.hist(1).n);
[firstFrame, lastFrame] = rts_frame_bounds(s);
if primaryIndex < firstFrame || primaryIndex > lastFrame
    primaryIndex = firstFrame;
    s.i = firstFrame;
    guidata(fig, s);
end
referenceTime = s.hist(1).now(primaryIndex);
for k = 1:numel(s.hist)
    if k == 1
        historyIndex = primaryIndex;
    else
        historyIndex = rts_nearest_row(s.hist(k).now, referenceTime);
    end
    x = s.hist(k).T(historyIndex, :);
    set(s.hVx(k), 'XData', x, 'YData', s.hist(k).Vx(historyIndex, :));
    set(s.hAx(k), 'XData', x, 'YData', s.hist(k).Ax(historyIndex, :));

end
if s.playing
    status = 'PLAY';
else
    status = 'pausa';
end
title(s.axVx, sprintf('frame %d/%d | replay %d:%d | slot %d | obs id %d | %s', ...
    s.i, s.n, firstFrame, lastFrame, s.slot, s.hist(1).ids(primaryIndex), status));
drawnow limitrate;
end

function [firstFrame, lastFrame] = rts_frame_bounds(state)
% Converte l'intervallo X corrente nei limiti, in frame, del replay.
timeLimits = xlim(state.axVx);
frameTimes = state.hist(1).now(:);
frameNumbers = (1:numel(frameTimes))';
validFrames = frameNumbers >= state.dataStartI & isfinite(frameTimes);
insideInterval = validFrames & frameTimes >= timeLimits(1) & frameTimes <= timeLimits(2);
selectedFrames = frameNumbers(insideInterval);
if isempty(selectedFrames)
    candidates = frameNumbers(validFrames);
    if isempty(candidates)
        firstFrame = state.dataStartI;
        lastFrame = state.dataStartI;
        return;
    end
    intervalCenter = mean(timeLimits);
    [~, nearest] = min(abs(frameTimes(candidates) - intervalCenter));
    firstFrame = candidates(nearest);
    lastFrame = firstFrame;
else
    firstFrame = selectedFrames(1);
    lastFrame = selectedFrames(end);
end
end

function history = rts_history_matrix(data, slot)
% Mantiene la forma [snapshot x profondita' history] anche con dimensioni singleton.
history = reshape(double(data(:, slot, :)), size(data, 1), size(data, 3));
end

function history = rts_mask_history(history, validSteps)
% Maschera le colonne non popolate di ogni snapshot.
for row = 1:size(history, 1)
    if validSteps(row) >= 0 && validSteps(row) < size(history, 2)
        history(row, validSteps(row)+1:end) = NaN;
    end
end
end

function [time, vx, ax] = rts_finalized_trace( ...
        timeHistory, vxHistory, axHistory, validSteps, maximumGap)
% Estrae da ogni snapshot l'ultimo campione valido del buffer originale.
numberOfRows = size(timeHistory, 1);
time = nan(numberOfRows, 1);
vx = nan(numberOfRows, 1);
ax = nan(numberOfRows, 1);
historyDepth = size(timeHistory, 2);
for row = 1:numberOfRows
    lastValid = floor(validSteps(row));
    if isfinite(lastValid) && lastValid >= 1 && lastValid <= historyDepth
        time(row) = timeHistory(row, lastValid);
        vx(row) = vxHistory(row, lastValid);
        ax(row) = axHistory(row, lastValid);
    end
end

% Durante il riempimento iniziale piu' snapshot aggiornano lo stesso
% campione: conserva soltanto la sua revisione finale.
valid = isfinite(time);
time = time(valid);
vx = vx(valid);
ax = ax(valid);
[time, finalRevision] = unique(time, 'last');
vx = vx(finalRevision);
ax = ax(finalRevision);
vx(vx == 0) = NaN;

% Impedisce a plot di collegare con rette campioni appartenenti a segmenti
% temporali separati, come gia' accade nella curva causale in presenza di NaN.
breakBefore = [false; diff(time) > maximumGap];
if any(breakBefore)
    outputIndex = (1:numel(time))' + cumsum(breakBefore);
    outputLength = numel(time) + nnz(breakBefore);
    timeWithGaps = nan(outputLength, 1);
    vxWithGaps = nan(outputLength, 1);
    axWithGaps = nan(outputLength, 1);
    timeWithGaps(outputIndex) = time;
    vxWithGaps(outputIndex) = vx;
    axWithGaps(outputIndex) = ax;
    time = timeWithGaps;
    vx = vxWithGaps;
    ax = axWithGaps;
end
end

function index = rts_nearest_row(nowVector, referenceTime)
% Trova la riga con timestamp "adesso" piu' vicino al log primario.
valid = find(isfinite(nowVector));
if isempty(valid)
    index = 1;
    return;
end
if ~isfinite(referenceTime)
    index = valid(1);
    return;
end
[~, nearest] = min(abs(nowVector(valid) - referenceTime));
index = valid(nearest);
end

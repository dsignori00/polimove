%% PSD_ERRORS
%  Welch: Hann window of NFFT samples, 50% overlap, each segment demeaned,
%  average over all segments of all stretches

% ---------------- CONFIG ----------------
SENS      = [1 4];            % indices in sensors{} (clustering, PointPillars)
SHOW_CM   = true;             % add the common mode (s1 + s2)/2
BETA      = 0.0058;           % [rad]
ERR_THR   = 5;                % [m] reject false associations
DT        = 0.1;              % [s] scan period
GAP_S     = 0.55;             % [s] gap that splits a stretch
NFFT      = 128;              % samples per segment (12.8 s, resolution 0.078 Hz)
F_SPLIT   = 2.0;              % [Hz] slow / fast split
MATCH_TOL = 0.02;             % [s] pairing tolerance for the common mode
CM_COL    = [0.06 0.17 0.33];

fs = 1/DT;

% ---------------- COMPENSATED ERRORS ----------------
ser = struct('name',{},'col',{},'t',{},'E',{});
for k = SENS
    [t,E] = comp_errors(sensors{k}.s, BETA, ERR_THR);
    ser(end+1) = struct('name',sensors{k}.name, 'col',sensors{k}.col, 't',t, 'E',E); %#ok<SAGROW>
end
if SHOW_CM && numel(ser) == 2
    [t2u,i2] = unique(ser(2).t);
    j = interp1(t2u, 1:numel(t2u), ser(1).t, 'nearest', NaN);
    v = isfinite(j) & abs(t2u(max(j,1)) - ser(1).t) < MATCH_TOL;
    j = i2(j(v));
    ser(end+1) = struct('name','common mode', 'col',CM_COL, 't',ser(1).t(v), ...
                        'E',(ser(1).E(v,:) + ser(2).E(j,:))/2);
end

% ---------------- FIGURE ----------------
figure('Name','PSD of compensated errors','Color','w','Position',[80 80 1200 460])
tiledlayout(1,2,'Padding','compact','TileSpacing','compact')
lab = {'longitudinal','lateral'};
fprintf('\nWelch PSD: NFFT %d (%.1f s, resolution %.3f Hz), Hann, 50%% overlap\n', NFFT, NFFT*DT, fs/NFFT);
for a = 1:2
    axp(a) = nexttile; hold on; grid on; box on %#ok<SAGROW>
    for r = 1:numel(ser)
        [fr,P,ns] = welch_psd(ser(r).t, ser(r).E(:,a), DT, GAP_S, NFFT);
        if isempty(P), continue, end
        share = sum(P(fr < F_SPLIT)) / sum(P);
        plot(fr, P, '-', 'Color',ser(r).col, 'LineWidth',1.6, 'DisplayName',ser(r).name)
        fprintf('%-14s %-12s %3d segments | power < %g Hz: %3.0f%%\n', ser(r).name, lab{a}, ns, F_SPLIT, 100*share);
    end
    set(gca,'YScale','log');  xlim([0 fs/2]);
    xline(F_SPLIT, '--k', sprintf('%g Hz',F_SPLIT), 'LineWidth',1.2, ...
        'LabelVerticalAlignment','bottom', 'HandleVisibility','off')
    xlabel('frequency [Hz]');  ylabel([lab{a} ' error PSD [m^2/Hz]']);
    legend('Location','northeast');
end
linkaxes(axp,'y');
fprintf('white-noise reference: %.0f%% of the power below %g Hz\n', 100*F_SPLIT/(fs/2), F_SPLIT);

% ---------------- LOCAL FUNCTIONS ----------------
function [t,E] = comp_errors(S, beta, err_thr)
    % ego-frame error after the geometric bias, sorted, gated, without duplicates
    t  = double(S.sens_stamp(:));
    ex = double(S.x_map_err(:));  ey = double(S.y_map_err(:));
    xr = double(S.x_rel(:));      yr = double(S.y_rel(:));
    pt = double(S.yaw_map(:));    pe = pt - double(S.yaw_rel(:));
    rho = hypot(xr,yr);  los = pe + atan2d(yr,xr);
    g = isfinite(t) & isfinite(ex) & isfinite(ey) & isfinite(rho) & ...
        isfinite(pt) & isfinite(pe) & hypot(ex,ey) < err_thr;
    A = [cosd(pt(g)), -sind(los(g));  sind(pt(g)), cosd(los(g))];
    y = [ex(g) + beta*sind(los(g)).*rho(g);  ey(g) - beta*cosd(los(g)).*rho(g)];
    p = A \ y;                                       % [D_LONG; C_LAT]
    lat = rho*beta + p(2);
    ex = ex - (cosd(pt)*p(1) - sind(los).*lat);
    ey = ey - (sind(pt)*p(1) + cosd(los).*lat);
    E  = [cosd(pe).*ex + sind(pe).*ey, -sind(pe).*ex + cosd(pe).*ey];   % map -> ego
    [t,o] = sort(t(g));  E = E(g,:);  E = E(o,:);
    keep = [true; diff(t) > 1e-3];
    t = t(keep);  E = E(keep,:);
end

function [fr,P,ns] = welch_psd(t, x, dt, gap_s, nfft)
    % Welch over continuous stretches, one-sided PSD [unit^2/Hz]
    t = t(:);  x = x(:);  m = isfinite(t) & isfinite(x);  t = t(m);  x = x(m);
    fr = [];  P = [];  ns = 0;
    w   = 0.5 - 0.5*cos(2*pi*(0:nfft-1)'/(nfft-1));          % Hann
    acc = zeros(floor(nfft/2)+1, 1);
    seg = [0; cumsum(diff(t) > gap_s)];
    for s = unique(seg)'
        ts = t(seg == s);  xs = x(seg == s);
        if ts(end) - ts(1) < (nfft-1)*dt, continue, end
        tg = (ts(1):dt:ts(end))';
        xg = interp1(ts, xs, tg, 'linear');
        for i0 = 1:floor(nfft/2):(numel(xg) - nfft + 1)
            xx  = xg(i0:i0+nfft-1);  xx = xx - mean(xx);
            X   = fft(w .* xx);
            acc = acc + abs(X(1:floor(nfft/2)+1)).^2;
            ns  = ns + 1;
        end
    end
    if ns == 0, return, end
    P = acc * dt / (ns * sum(w.^2));
    P(2:end-1) = 2*P(2:end-1);
    fr = (0:floor(nfft/2))' / (nfft*dt);
end
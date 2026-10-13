%% R_estimate
%
% For each sensor
%   1) Gaussian fit (2 subplots: ex, ey)
%   2) Confidence ellipse 1sigma and 2sigma in the (ex, ey) plane
%
% Command window: table with sigma_x, sigma_y, R_xx, R_yy, sigma_yaw, R_yaw,
% sigma_v, R_vv

if ~exist('ax', 'var'); ax = gobjects(0); end

if ~exist('sensors', 'var')
    error('R_time_analysis: variable "sensors" not found.');
end
if ~exist('gt', 'var')
    error('R_time_analysis: variable "gt" not found.');
end
if ~exist('err_thr', 'var')
    err_thr = 10;
    warning('R_time_analysis: err_thr not found, using %.1f.', err_thr);
end
if ~exist('sz', 'var'); sz = 3; end

pos_thr = 5.0;
v_thr   = 2.0;
yaw_thr = 30.0;

% Use every configured analysis source, including target tracking when enabled.
sensors_r = sensors;

fprintf('\n===== R TIME ANALYSIS =====\n');
fprintf('%-20s  %10s  %10s  %10s  %10s  %10s  %10s  %10s  %10s\n', ...
    'Sensor', 'sigma_x', 'sigma_y', 'R_xx', 'R_yy', 'sigma_yaw', 'R_yaw', 'sigma_v', 'R_vv');
fprintf('%s\n', repmat('-', 1, 112));

gaussian_fig = figure('Name','Noise Fit','NumberTitle','off', 'WindowStyle','docked','Color','w');
gaussian_tabs = uitabgroup(gaussian_fig);

for k = 1:numel(sensors_r)

    s        = sensors_r{k}.s;
    col_k    = sensors_r{k}.col;
    name     = sensors_r{k}.name;
    has_rho_dot = sensors_r{k}.has_rho_dot;

    % Display name for tables and figure titles
    switch name
        case 'lidar';  display_name = 'LIDAR CLUSTERING';
        case 'pp';     display_name = 'LIDAR POINTPILLARS';
        case 'radar';  display_name = 'RADAR';
        case 'camera'; display_name = 'CAMERA';
        case 'target tracking'; display_name = 'TARGET TRACKING';
        otherwise;     display_name = name;
    end

    %% ----------------------------------------------------------
    % 0. Position gating
    % ----------------------------------------------------------
    x_map_err  = s.x_map_err;
    y_map_err  = s.y_map_err;
    n_cols     = size(x_map_err, 2);
    sens_stamp = repmat(s.sens_stamp, 1, n_cols);

    x_rel_err_all = s.x_rel_err;
    y_rel_err_all = s.y_rel_err;
    yaw_map_err_all = s.yaw_map_err;
    if size(x_rel_err_all, 2) < n_cols
        x_rel_err_all = repmat(x_rel_err_all(:,1), 1, n_cols);
        y_rel_err_all = repmat(y_rel_err_all(:,1), 1, n_cols);
    end
    if size(yaw_map_err_all, 2) < n_cols
        yaw_map_err_all = repmat(yaw_map_err_all(:,1), 1, n_cols);
    end

    dist_pos  = hypot(x_rel_err_all, y_rel_err_all);
    gate_mask = dist_pos < pos_thr;

    x_map_err  = x_map_err(:);
    y_map_err  = y_map_err(:);
    yaw_map_err = yaw_map_err_all(:);
    sens_stamp = sens_stamp(:);
    gate_mask  = gate_mask(:);

    valid = ~isnan(x_map_err) & ~isnan(y_map_err) & ...
            ~isnan(sens_stamp) & gate_mask;

    x_map_err  = x_map_err(valid);
    y_map_err  = y_map_err(valid);
    yaw_map_err = yaw_map_err(valid);
    sens_stamp = sens_stamp(valid);

    if numel(sens_stamp) < 10
        warning('R_time_analysis: %s too few samples after gating. Skip.', name);
        continue;
    end

    %% ----------------------------------------------------------
    % 1. Rotation to relative frame
    % ----------------------------------------------------------
    [gt_stamp_u, ui_gt] = unique(gt.stamp(:));
    yaw_gt_u = gt.yaw_map(ui_gt);
    psi_gt   = interp1(gt_stamp_u, yaw_gt_u, sens_stamp, 'linear', nan);

    valid_gt   = ~isnan(psi_gt);
    x_map_err  = x_map_err(valid_gt);
    y_map_err  = y_map_err(valid_gt);
    yaw_map_err = yaw_map_err(valid_gt);
    sens_stamp = sens_stamp(valid_gt);
    psi_gt     = psi_gt(valid_gt);

    psi_gt_rad = deg2rad(psi_gt);
    x_err_rel =  cos(psi_gt_rad) .* x_map_err + sin(psi_gt_rad) .* y_map_err;
    y_err_rel = -sin(psi_gt_rad) .* x_map_err + cos(psi_gt_rad) .* y_map_err;

    ass = hypot(x_err_rel, y_err_rel) < err_thr;
    x_err_rel(~ass) = nan;
    y_err_rel(~ass) = nan;
    yaw_map_err(~ass | abs(yaw_map_err) >= yaw_thr) = nan;

    %% ----------------------------------------------------------
    % 2. Global Gaussian fit
    % ----------------------------------------------------------
    % Coppie (ex,ey) ALLINEATE: entrambi validi, cosi il fit e la heatmap
    % usano esattamente gli stessi campioni (il centro dell'ellisse coincide
    % con la nuvola di densita).
    both_valid = ~isnan(x_err_rel) & ~isnan(y_err_rel);
    ex = x_err_rel(both_valid);
    ey = y_err_rel(both_valid);
    eyaw = yaw_map_err(~isnan(yaw_map_err));

    pd_x    = fitdist(ex, 'Normal');
    pd_y    = fitdist(ey, 'Normal');
    sigma_x = pd_x.sigma;  mu_x = pd_x.mu;
    sigma_y = pd_y.sigma;  mu_y = pd_y.mu;
    R_xx    = sigma_x^2;
    R_yy    = sigma_y^2;

    sigma_yaw = nan;  R_yaw = nan;  mu_yaw = nan;
    if numel(eyaw) >= 10
        pd_yaw    = fitdist(eyaw, 'Normal');
        sigma_yaw = pd_yaw.sigma;
        mu_yaw    = pd_yaw.mu;
        R_yaw     = sigma_yaw^2;
    end

    %% ----------------------------------------------------------
    % 3. rho_dot for sources that provide it — fit on precomputed rho_dot_err
    % ----------------------------------------------------------
    sigma_vv = nan;  R_vv = nan;  mu_vv = nan;
    if has_rho_dot && isfield(s, 'rho_dot_err')
        e_vv = s.rho_dot_err(:, 1);
        valid_vv = ~isnan(e_vv) & abs(e_vv) < v_thr;
        e_vv = e_vv(valid_vv);
        if numel(e_vv) >= 10
            pd_vv    = fitdist(e_vv, 'Normal');
            sigma_vv = pd_vv.sigma;
            mu_vv    = pd_vv.mu;
            R_vv     = sigma_vv^2;
        end
    end

    % Command window table
    if has_rho_dot && ~isnan(sigma_vv)
        fprintf('%-20s  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f\n', ...
            display_name, sigma_x, sigma_y, R_xx, R_yy, sigma_yaw, R_yaw, sigma_vv, R_vv);
    elseif has_rho_dot
        fprintf('%-20s  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10s  %10s\n', ...
            display_name, sigma_x, sigma_y, R_xx, R_yy, sigma_yaw, R_yaw, 'n/a', 'n/a');
    else
        fprintf('%-20s  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10.4f  %10s  %10s\n', ...
            display_name, sigma_x, sigma_y, R_xx, R_yy, sigma_yaw, R_yaw, '-', '-');
    end

    %% ----------------------------------------------------------
    % FIG 1: Gaussian fit
    % ----------------------------------------------------------
    sensor_tab = uitab(gaussian_tabs,'Title',display_name);
    sensor_tabs = uitabgroup(sensor_tab);
    fit_tab = uitab(sensor_tabs,'Title','pdf');
    ellipse_tab = uitab(sensor_tabs,'Title','ellipse');
    tl_g = tiledlayout(fit_tab,1,3,'Padding','compact','TileSpacing','loose');

    err_ch = {ex,      ey,      eyaw};
    mu_ch  = {mu_x,    mu_y,    mu_yaw};
    sig_ch = {sigma_x, sigma_y, sigma_yaw};
    xlbl_g = {'e_x  [m]', 'e_y  [m]', 'e_\psi  [deg]'};
    ttl_g  = {'Longitudinal error', 'Lateral error', 'Yaw error'};

    for d = 1:3
        ax_g = nexttile(tl_g);
        hold(ax_g,'on'); grid(ax_g,'on'); box(ax_g,'on');

        mu_d  = mu_ch{d};
        sig_d = sig_ch{d};
        e_d   = err_ch{d};

        if numel(e_d) < 10 || isnan(sig_d)
            title(ax_g, sprintf('%s - insufficient samples', ttl_g{d}), 'FontSize', 10);
            xlabel(ax_g, xlbl_g{d}, 'Interpreter','tex', 'FontSize', 10);
            ylabel(ax_g, 'pdf', 'FontSize', 10);
            continue;
        end

        histogram(e_d, 50, 'Normalization','pdf', ...
            'FaceColor', col_k, 'FaceAlpha', 0.35, ...
            'EdgeColor', 'none', 'Parent', ax_g, 'HandleVisibility','off');

        x_fit   = linspace(mu_d - 4*sig_d, mu_d + 4*sig_d, 300);
        pdf_fit = normpdf(x_fit, mu_d, sig_d);
        pdf_max = max(pdf_fit);

        plot(ax_g, x_fit, pdf_fit, '-', ...
            'Color', col_k, 'LineWidth', 2.2, 'HandleVisibility','off');

        xline(ax_g, mu_d, '-', 'Color', col_k*0.7, 'LineWidth', 1.4, ...
            'Label', '\mu', 'Interpreter', 'tex', ...
            'LabelVerticalAlignment','bottom', 'HandleVisibility','off');
        xline(ax_g, mu_d+sig_d, '--', 'Color', col_k*0.7, 'LineWidth', 1.1, ...
            'Label', '+\sigma', 'Interpreter', 'tex', ...
            'LabelVerticalAlignment','bottom', 'HandleVisibility','off');
        xline(ax_g, mu_d-sig_d, '--', 'Color', col_k*0.7, 'LineWidth', 1.1, ...
            'Label', '-\sigma', 'Interpreter', 'tex', ...
            'LabelVerticalAlignment','bottom', 'HandleVisibility','off');

        step = max(1, floor(numel(e_d)/800));
        plot(ax_g, e_d(1:step:end), ...
            repmat(-0.03*pdf_max, numel(e_d(1:step:end)), 1), ...
            '.', 'Color', col_k, 'MarkerSize', 3, 'HandleVisibility','off');

        xlim(ax_g, [mu_d - 4*sig_d,  mu_d + 4*sig_d]);
        ylim(ax_g, [-0.08*pdf_max,    pdf_max*1.18]);
        xlabel(ax_g, xlbl_g{d}, 'Interpreter','tex', 'FontSize', 10);
        ylabel(ax_g, 'pdf', 'FontSize', 10);
        title(ax_g,  ttl_g{d}, 'FontSize', 10);
    end

    sgtitle(tl_g, sprintf('%s  -  Gaussian fit - position error', upper(display_name)), ...
        'FontSize', 12, 'FontWeight', 'bold', 'Interpreter', 'none');

    %% ----------------------------------------------------------
    % FIG 2: Confidence ellipse + density heatmap
    % ----------------------------------------------------------
    tl_e = tiledlayout(ellipse_tab,1,1,'Padding','compact');
    ax_e = nexttile(tl_e);
    hold(ax_e,'on'); grid(ax_e,'on'); box(ax_e,'on'); axis(ax_e,'equal');
    ax_e.Layer = 'top';                 % griglia sopra la heatmap
    ax_e.GridColor = [0.4 0.4 0.4];
    ax_e.GridAlpha = 0.5;

    % --- Heatmap di densità 2D (KDE su griglia) ---
    % Limiti coerenti con la vista finale
    lim_val = 2.3 * max(sigma_x, sigma_y);
    xr = mu_x + [-lim_val lim_val];
    yr = mu_y + [-lim_val lim_val];

    % ex, ey sono gia le coppie allineate (stesse usate nel fit)
    ex_p = ex;
    ey_p = ey;

    % Griglia per la densità
    ngrid = 120;
    xg = linspace(xr(1), xr(2), ngrid);
    yg = linspace(yr(1), yr(2), ngrid);
    [XG, YG] = meshgrid(xg, yg);

    % KDE gaussiana: bandwidth con regola di Scott
    n_pts = numel(ex_p);
    if n_pts >= 10
        bw_x = 1.06 * std(ex_p) * n_pts^(-1/5);
        bw_y = 1.06 * std(ey_p) * n_pts^(-1/5);
        bw_x = max(bw_x, 1e-3);
        bw_y = max(bw_y, 1e-3);

        % Densità accumulata (vettorizzata a blocchi per non saturare la RAM)
        dens = zeros(size(XG));
        blk = 5000;
        for i0 = 1:blk:n_pts
            i1 = min(i0+blk-1, n_pts);
            exb = reshape(ex_p(i0:i1), 1, 1, []);
            eyb = reshape(ey_p(i0:i1), 1, 1, []);
            dens = dens + sum( ...
                exp(-0.5*((XG - exb)/bw_x).^2) .* ...
                exp(-0.5*((YG - eyb)/bw_y).^2), 3);
        end
        dens = dens / (n_pts * 2*pi*bw_x*bw_y);

        % Disegna la heatmap sotto tutto
        imagesc(ax_e, xg, yg, dens);
        set(ax_e, 'YDir', 'normal');

        % Colormap chiara->intensa basata sul colore del sensore
        ncol = 256;
        base = col_k(:)';
        white = [1 1 1];
        cmap = (1-linspace(0,1,ncol)').*white + linspace(0,1,ncol)'.*base;
        colormap(ax_e, cmap);

        cb = colorbar(ax_e);
        cb.Label.String = 'density';
        cb.Label.FontSize = 9;
    end

    theta = linspace(0, 2*pi, 500);

    % Ellisse 1sigma
    x1 = mu_x + sigma_x * cos(theta);
    y1 = mu_y + sigma_y * sin(theta);
    plot(ax_e, x1, y1, '--', ...
        'Color', [0 0 0], 'LineWidth', 2.2, 'DisplayName', '1\sigma');

    % Ellisse 2sigma
    x2 = mu_x + 2*sigma_x * cos(theta);
    y2 = mu_y + 2*sigma_y * sin(theta);
    plot(ax_e, x2, y2, '--', ...
        'Color', [0.25 0.25 0.25], 'LineWidth', 1.6, 'DisplayName', '2\sigma');

    % Media
    plot(ax_e, mu_x, mu_y, '+', ...
        'Color', [0.9 0.1 0.1], 'MarkerSize', 11, 'LineWidth', 2.2, ...
        'DisplayName', '\mu');

    xlabel(ax_e, 'e_x  [m]', 'Interpreter','tex', 'FontSize', 11);
    ylabel(ax_e, 'e_y  [m]', 'Interpreter','tex', 'FontSize', 11);
    title(ax_e, sprintf('%s  —  confidence ellipse', upper(display_name)), ...
        'FontSize', 12, 'FontWeight','bold', 'Interpreter','none');
    legend(ax_e, 'show', 'Location','northeast', ...
        'Interpreter','tex', 'FontSize', 10, 'Box','on', 'Color','w');

    xlim(ax_e, xr);
    ylim(ax_e, yr);


end

fprintf('%s\n\n', repmat('-', 1, 112));

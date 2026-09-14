% CPR_PSIREL - correlation between radar vx pseudo-measurement error and
% psi_rel (aspect angle).
%
% psi_rel here is the ASPECT ANGLE, i.e. the angle between the target's
% relative heading (yaw_rel) and the line of sight (bearing):
%
%   psi_rel = wrapToPi(yaw_rel - atan2(y_rel, x_rel))
%
% This is NOT yaw_rel alone: yaw_rel is the target heading in the ego
% frame, while the angle that actually governs the Doppler projection
% (rho_dot -> vx) is the angle between target heading and line of sight,
% which coincides with yaw_rel only if the target is exactly ahead of you
% (bearing = 0). Same convention already used for the radar overlay in
% rls.m (rls_aspect).
%
% SCRIPT (not a function): reads 'log', 'gt', 'rad_clust' from the
% workspace, same convention as rls.m.

%% SECTION 0 - PARAMETERS
cpr_slot     = 1;
cpr_cosMin   = 0.15;   % |cos(psi_rel)| minimum, same as rls_cosMin
cpr_rhoSign  = 1;      % same as rls_rhoSign
cpr_windows  = [];     % e.g. [32 34; 37 40]; empty = whole log (relative time, as rls_t*)
cpr_errMax   = 5;      % [m/s] outlier gate: |error| > cpr_errMax is discarded

% ---------- sanity checks ----------
if ~exist('rad_clust','var'), error('cpr: "rad_clust" not found in workspace.'); end
if ~exist('gt','var'),        error('cpr: "gt" not found in workspace.'); end
if ~exist('log','var'),       error('cpr: "log" not found in workspace.'); end

cpr_s = rad_clust;

%% SECTION 1 - COMMON TIME REFERENCE (same as rls.m)
cpr_t0 = NaN;
if isfield(log,'perception__opponents') && isfield(log.perception__opponents,'stamp__tot')
    cpr_t0 = min(double(log.perception__opponents.stamp__tot), [], 'omitnan');
end
if isnan(cpr_t0), error('cpr: cannot determine t0 (log.perception__opponents.stamp__tot).'); end

%% SECTION 2 - FIELD EXTRACTION FROM rad_clust (slot 1, as in rls.m)
cpr_rd   = double(cpr_s.rho_dot(:,cpr_slot));  cpr_rd(cpr_rd==0)   = NaN;
cpr_xr   = double(cpr_s.x_rel(:,cpr_slot));    cpr_xr(cpr_xr==0)   = NaN;
cpr_yr   = double(cpr_s.y_rel(:,cpr_slot));    cpr_yr(cpr_yr==0)   = NaN;
cpr_yawr = double(cpr_s.yaw_rel(:,cpr_slot));
if max(abs(cpr_yawr), [], 'omitnan') > 2*pi
    cpr_yawr = deg2rad(cpr_yawr);
end
cpr_yawr = wrapToPi(cpr_yawr);

cpr_sensStamp = double(cpr_s.sens_stamp(:,cpr_slot)); cpr_sensStamp(cpr_sensStamp==0) = NaN;

% ---------- psi_rel = aspect angle ----------
cpr_beta   = atan2(cpr_yr, cpr_xr);
cpr_psirel = wrapToPi(cpr_yawr - cpr_beta);
cpr_c      = cos(cpr_psirel);
cpr_c(abs(cpr_c) < cpr_cosMin) = NaN;
cpr_ampl   = 1 ./ abs(cos(cpr_psirel));   % geometric amplification factor, for plotting

%% SECTION 3 - EGO-MOTION COMPENSATION (same as rls.m)
if isfield(log,'estimation') && all(isfield(log.estimation, {'stamp__tot','vx'}))
    cpr_teRaw    = double(log.estimation.stamp__tot(:));
    cpr_egoVxAll = double(log.estimation.vx(:));
    cpr_egoOk    = isfinite(cpr_teRaw) & isfinite(cpr_egoVxAll);

    cpr_vegoI = interp1(cpr_teRaw(cpr_egoOk), cpr_egoVxAll(cpr_egoOk), ...
        cpr_sensStamp, 'linear', 'extrap');

    cpr_vxMeas = (cpr_rhoSign * cpr_rd + cpr_vegoI .* cos(cpr_beta)) ./ cpr_c;
else
    warning('cpr: log.estimation.vx not found, no ego compensation applied.');
    cpr_vxMeas = (cpr_rhoSign * cpr_rd) ./ cpr_c;
end

%% SECTION 4 - GT INTERPOLATION AND ERROR
cpr_gtT = double(gt.stamp) - cpr_t0;
cpr_gtV = double(gt.vx);
cpr_measT = cpr_sensStamp - cpr_t0;

cpr_vxGt = interp1(cpr_gtT, cpr_gtV, cpr_measT, 'linear', NaN);
cpr_err  = cpr_vxMeas - cpr_vxGt;

%% SECTION 5 - VALIDITY MASK: finite values, error gate, time windows
cpr_valid = isfinite(cpr_err) & isfinite(cpr_psirel);
fprintf('\nFinite samples (pre-gate)    : %d out of %d\n', nnz(cpr_valid), numel(cpr_err));

if ~isempty(cpr_errMax)
    cpr_gated = cpr_valid & abs(cpr_err) > cpr_errMax;
    fprintf('Discarded by gate |err|>%.1f m/s : %d\n', cpr_errMax, nnz(cpr_gated));
    cpr_valid = cpr_valid & abs(cpr_err) <= cpr_errMax;
end

if ~isempty(cpr_windows)
    cpr_inWin = false(size(cpr_measT));
    for cpr_w = 1:size(cpr_windows,1)
        cpr_inWin = cpr_inWin | (cpr_measT >= cpr_windows(cpr_w,1) & cpr_measT <= cpr_windows(cpr_w,2));
    end
    cpr_valid = cpr_valid & cpr_inWin;
end

cpr_tv   = cpr_measT(cpr_valid);
cpr_ev   = cpr_err(cpr_valid);
cpr_cv   = cpr_c(cpr_valid);
cpr_prv  = rad2deg(cpr_psirel(cpr_valid));
cpr_amplv = cpr_ampl(cpr_valid);

fprintf('Valid samples (final)        : %d out of %d\n', nnz(cpr_valid), numel(cpr_err));
if nnz(cpr_valid) < 10
    warning('cpr: very few valid samples, check cosMin/windows/sign/gate.');
end

%% SECTION 6 - CORRELATIONS AND STATISTICS
fprintf('\n=== CORRELATIONS ===\n');

[cpr_rP, cpr_pP] = corr(abs(cpr_prv), abs(cpr_ev), 'type','Pearson',  'rows','complete');
[cpr_rS, cpr_pS] = corr(abs(cpr_prv), abs(cpr_ev), 'type','Spearman', 'rows','complete');
fprintf('|err| vs |psi_rel|   : Pearson r=%+.3f (p=%.2e)  Spearman r=%+.3f (p=%.2e)\n', ...
    cpr_rP, cpr_pP, cpr_rS, cpr_pS);

[cpr_rA, cpr_pA] = corr(cpr_amplv, abs(cpr_ev), 'type','Spearman', 'rows','complete');
fprintf('|err| vs 1/|cos|     : Spearman r=%+.3f (p=%.2e)\n', cpr_rA, cpr_pA);

[cpr_rC, cpr_pC] = corr(abs(cpr_cv), abs(cpr_ev), 'type','Spearman', 'rows','complete');
fprintf('|err| vs |cos|       : Spearman r=%+.3f (p=%.2e)  (expected NEGATIVE)\n', cpr_rC, cpr_pC);

fprintf('\nGlobal RMSE (radar)  : %.3f m/s\n', sqrt(mean(cpr_ev.^2,'omitnan')));
fprintf('Mean bias            : %+.3f m/s\n', mean(cpr_ev,'omitnan'));

% ---------- RMSE per |psi_rel| bin ----------
cpr_edges = 0:15:90;
cpr_nb = numel(cpr_edges)-1;
cpr_rmseBin = nan(cpr_nb,1); cpr_biasBin = nan(cpr_nb,1);
cpr_nBin = zeros(cpr_nb,1); cpr_ctr = nan(cpr_nb,1);
for cpr_b = 1:cpr_nb
    cpr_sel = abs(cpr_prv) >= cpr_edges(cpr_b) & abs(cpr_prv) < cpr_edges(cpr_b+1);
    cpr_nBin(cpr_b) = nnz(cpr_sel);
    cpr_ctr(cpr_b)  = mean(cpr_edges(cpr_b:cpr_b+1));
    if cpr_nBin(cpr_b) > 2
        cpr_rmseBin(cpr_b) = sqrt(mean(cpr_ev(cpr_sel).^2,'omitnan'));
        cpr_biasBin(cpr_b) = mean(cpr_ev(cpr_sel),'omitnan');
    end
end
fprintf('\n=== RMSE PER |psi_rel| BIN ===\n');
fprintf('%8s %8s %10s %10s\n','bin[deg]','N','RMSE','bias');
for cpr_b = 1:cpr_nb
    fprintf('%3d-%-4d %8d %10.3f %10.3f\n', cpr_edges(cpr_b), cpr_edges(cpr_b+1), ...
        cpr_nBin(cpr_b), cpr_rmseBin(cpr_b), cpr_biasBin(cpr_b));
end

%% SECTION 7 - PLOTS
% Figure 1: scatter of error vs psi_rel, colored by time
figure('Name','Radar vx error vs relative yaw - scatter','Color','w');
scatter(cpr_prv, cpr_ev, 18, cpr_tv, 'filled'); grid on; hold on;
yline(0,'k--');
cb = colorbar; cb.Label.String = 'time [s]';
xlabel('\psi_{rel} (aspect angle) [deg]'); ylabel('radar v_x error [m/s]');
title(sprintf('Error vs \\psi_{rel}  (Spearman |err|/|\\psi_{rel}| = %+.2f)', cpr_rS));

% Figure 2: RMSE per |psi_rel| bin
figure('Name','Radar vx error vs relative yaw - RMSE per bin','Color','w');
yyaxis left;  bar(cpr_ctr, cpr_rmseBin); ylabel('RMSE [m/s]');
yyaxis right; plot(cpr_ctr, cpr_nBin, 'o-'); ylabel('N samples');
grid on; xlabel('|\psi_{rel}| [deg]');
title('RMSE per \psi_{rel} bin');

%% SECTION 8 - CONFOUNDING CHECK
% psi_rel and dynamics (braking/acceleration) can covary: in a curve you
% often have both large psi_rel and large |ax|. Without separating the
% two effects, a strong correlation between error and psi_rel could be
% wrongly attributed to geometry when it is actually driven by dynamics.
if isfield(log,'perception__opponents') && isfield(log.perception__opponents,'opponents__ax')
    cpr_axAll = double(log.perception__opponents.opponents__ax(:,cpr_slot));
    cpr_axT   = double(log.perception__opponents.stamp__tot) - cpr_t0;
    cpr_axI   = interp1(cpr_axT, cpr_axAll, cpr_tv, 'linear', NaN);
    cpr_okAx  = isfinite(cpr_axI);
    if nnz(cpr_okAx) > 10
        [cpr_rAx, cpr_pAx] = corr(abs(cpr_axI(cpr_okAx)), abs(cpr_ev(cpr_okAx)), 'type','Spearman');
        fprintf('\n=== CONFOUNDING ===\n');
        fprintf('|err| vs |ax|        : Spearman r=%+.3f (p=%.2e)\n', cpr_rAx, cpr_pAx);
        [cpr_rXc, cpr_pXc] = corr(abs(cpr_axI(cpr_okAx)), abs(cpr_prv(cpr_okAx)), 'type','Spearman');
        fprintf('|ax|  vs |psi_rel|   : Spearman r=%+.3f (p=%.2e)\n', cpr_rXc, cpr_pXc);
        fprintf(['If |ax| and |psi_rel| are strongly correlated with each other,\n' ...
                 'stratify: repeat the analysis on subsets with roughly constant |ax|.\n']);
    end
end

clear cpr_s cpr_rd cpr_xr cpr_yr cpr_yawr cpr_sensStamp cpr_beta cpr_c cpr_ampl
clear cpr_teRaw cpr_egoVxAll cpr_egoOk cpr_vegoI cpr_gtT cpr_gtV cpr_measT
clear cpr_inWin cpr_w cpr_gated cpr_b cpr_sel cpr_axAll cpr_axT cpr_axI cpr_okAx
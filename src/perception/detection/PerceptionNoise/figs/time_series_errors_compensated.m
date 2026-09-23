%% TIME_SERIES_ERRORS_COMPENSATED
% Errori originali e compensati nei frame MAP ed EGO.
if ~show_error_series
    return
end

%% CONFIG
COMP_MODE = 'estimate';     % 'estimate' | 'workspace' | 'none'
BETA      = 0.0058;         % [rad]

if ~exist('err_thr','var') || isempty(err_thr), err_thr = 5; end
if ~exist('f','var')       || isempty(f),       f = 1; end
if ~exist('y_err_lim','var') || isempty(y_err_lim), y_err_lim = [-1 1]; end

c_orig = [0.65 0.65 0.65];

%% ============================================================
% PRECOMPUTE ERRORS
% ============================================================

ERR = cell(size(sensors));

for i = 1:numel(sensors)

    S = sensors{i}.s;

    t       = double(S.sens_stamp(:));
    ex      = double(S.x_map_err(:));
    ey      = double(S.y_map_err(:));
    xr      = double(S.x_rel(:));
    yr      = double(S.y_rel(:));
    psi_tgt = double(S.yaw_map(:));
    psi_ego = psi_tgt - double(S.yaw_rel(:));

    rho = hypot(xr,yr);
    los = psi_ego + atan2d(yr,xr);

    gate = ...
        isfinite(t) & ...
        isfinite(ex) & ...
        isfinite(ey) & ...
        isfinite(rho) & ...
        isfinite(psi_tgt) & ...
        isfinite(psi_ego) & ...
        hypot(ex,ey) < err_thr;

    %% --- Bias model

    mode = lower(COMP_MODE);

    if strcmp(mode,'workspace') && ...
       ~(exist('PAR','var') && isstruct(PAR) && ...
         all(isfield(PAR,{'BETA','D_LONG','C_LAT'})))
        mode = 'estimate';
    end

    switch mode

        case 'workspace'
            P = PAR;

        case 'estimate'
            P.BETA = BETA;

            ct = cosd(psi_tgt);
            st = sind(psi_tgt);
            cl = cosd(los);
            sl = sind(los);

            A = [ ...
                ct(gate), -sl(gate);
                st(gate),  cl(gate)];

            y = [ ...
                ex(gate) + P.BETA.*sl(gate).*rho(gate);
                ey(gate) - P.BETA.*cl(gate).*rho(gate)];

            p = A \ y;

            P.D_LONG = p(1);
            P.C_LAT  = p(2);

        case 'none'
            P = struct('BETA',0,'D_LONG',0,'C_LAT',0);

        otherwise
            error('COMP_MODE non riconosciuto.');
    end

    fprintf('%s: D_LONG %+.3f m | C_LAT %+.3f m | BETA %+.3f deg\n', ...
        sensors{i}.name, P.D_LONG, P.C_LAT, rad2deg(P.BETA));

    %% --- Compensation in MAP frame

    [dx,dy] = bias_model(psi_tgt,los,rho,P);

    ex_c = ex - dx;
    ey_c = ey - dy;

    %% --- Original error in EGO frame

    if isfield(S,'x_rel_err') && isfield(S,'y_rel_err')
        ex_rel = double(S.x_rel_err(:));
        ey_rel = double(S.y_rel_err(:));
    else
        [ex_rel,ey_rel] = map2frame(ex,ey,psi_ego);
    end

    %% --- Compensation in EGO frame

    [dx_rel,dy_rel] = map2frame(dx,dy,psi_ego);

    ex_rel_c = ex_rel - dx_rel;
    ey_rel_c = ey_rel - dy_rel;

    %% --- Store

    ERR{i}.t = t;
    ERR{i}.gate = gate;

    ERR{i}.x_map     = ex;
    ERR{i}.y_map     = ey;
    ERR{i}.x_map_c   = ex_c;
    ERR{i}.y_map_c   = ey_c;

    ERR{i}.x_rel     = ex_rel;
    ERR{i}.y_rel     = ey_rel;
    ERR{i}.x_rel_c   = ex_rel_c;
    ERR{i}.y_rel_c   = ey_rel_c;
end


%% ============================================================
% X MAP
% ============================================================

figure('Name','Error - x map','Color','w')
tiledlayout(numel(sensors),1,'Padding','compact')

for i = 1:numel(sensors)

    ax(f) = nexttile;
    hold on; grid on; box on
    f = f+1;

    E = ERR{i};
    m = E.gate;

    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')

    plot(E.t(m),E.x_map(m),'.', ...
        'Color',c_orig,'MarkerSize',9,'DisplayName','Original');

    plot(E.t(m),E.x_map_c(m),'.', ...
        'Color',sensors{i}.col,'MarkerSize',9,'DisplayName','Compensated');

    ylabel('error [m]')
    title(sensors{i}.name)
    ylim(y_err_lim)
    xlim([0 inf])
    legend show
end

xlabel('timestamp [s]');


%% ============================================================
% Y MAP
% ============================================================

figure('Name','Error - y map','Color','w')
tiledlayout(numel(sensors),1,'Padding','compact')

for i = 1:numel(sensors)

    ax(f) = nexttile;
    hold on; grid on; box on
    f = f+1;

    E = ERR{i};
    m = E.gate;

    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')

    plot(E.t(m),E.y_map(m),'.', ...
        'Color',c_orig,'MarkerSize',9,'DisplayName','Original');

    plot(E.t(m),E.y_map_c(m),'.', ...
        'Color',sensors{i}.col,'MarkerSize',9,'DisplayName','Compensated');

    ylabel('error [m]')
    title(sensors{i}.name)
    ylim(y_err_lim)
    xlim([0 inf])
    legend show
end

xlabel('timestamp [s]');


%% ============================================================
% X RELATIVE
% ============================================================

figure('Name','Error - x relative','Color','w')
tiledlayout(numel(sensors),1,'Padding','compact')

for i = 1:numel(sensors)

    ax(f) = nexttile;
    hold on; grid on; box on
    f = f+1;

    E = ERR{i};
    m = E.gate;

    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')

    plot(E.t(m),E.x_rel(m),'.', ...
        'Color',c_orig,'MarkerSize',9,'DisplayName','Original');

    plot(E.t(m),E.x_rel_c(m),'.', ...
        'Color',sensors{i}.col,'MarkerSize',9,'DisplayName','Compensated');

    ylabel('error [m]')
    title(sensors{i}.name)
    ylim(y_err_lim)
    xlim([0 inf])
    legend show
end

xlabel('timestamp [s]');


%% ============================================================
% Y RELATIVE
% ============================================================

figure('Name','Error - y relative','Color','w')
tiledlayout(numel(sensors),1,'Padding','compact')

for i = 1:numel(sensors)

    ax(f) = nexttile;
    hold on; grid on; box on
    f = f+1;

    E = ERR{i};
    m = E.gate;

    yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')

    plot(E.t(m),E.y_rel(m),'.', ...
        'Color',c_orig,'MarkerSize',9,'DisplayName','Original');

    plot(E.t(m),E.y_rel_c(m),'.', ...
        'Color',sensors{i}.col,'MarkerSize',9,'DisplayName','Compensated');

    ylabel('error [m]')
    title(sensors{i}.name)
    ylim(y_err_lim)
    xlim([0 inf])
    legend show
end

xlabel('timestamp [s]');


%% ============================================================
% LOCAL FUNCTIONS
% ============================================================

function [dx,dy] = bias_model(psi_tgt,los,rho,P)

ct = cosd(psi_tgt);
st = sind(psi_tgt);
cl = cosd(los);
sl = sind(los);

lat = rho.*P.BETA + P.C_LAT;

dx = ct.*P.D_LONG - sl.*lat;
dy = st.*P.D_LONG + cl.*lat;

dx(~isfinite(dx)) = 0;
dy(~isfinite(dy)) = 0;
end


function [a,b] = map2frame(ex,ey,psi)

c = cosd(psi(:));
s = sind(psi(:));

a =  c.*ex(:) + s.*ey(:);
b = -s.*ex(:) + c.*ey(:);
end
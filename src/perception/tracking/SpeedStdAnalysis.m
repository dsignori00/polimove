close all; clearvars -except tt tt2; clc;

compare = false;
%#ok<*UNRCH>
%#ok<*INUSD>

proj = currentProject();
addpath(fullfile(proj.RootFolder,'src','perception','utils'));

% style
set(0,'DefaultFigureWindowStyle','docked');
set(0,'DefaultTextInterpreter', 'none');
set(0,'DefaultLegendInterpreter', 'none');
set(0, 'DefaultLineLineWidth', 2);
col.tt = '#0072BD';
col.tt2 = '#7E2F8E';
sz=3; % Marker size
f = 1;
x_lim = [0 inf];

% load log
if (~exist('tt','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    load(fullfile(path,file), 'log');
    tt = load_tt(log);
end

% load log 2
if(compare)
    if (~exist('tt2','var'))
        [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log_2');
        if isequal(file, 0)  
        disp('User canceled file selection.');
        else
        tmp = load(fullfile(path,file), 'log');
        log_2 = tmp.log;
        clearvars tmp;
        end
    end
    name2 = 'New TT';
    tt2 = load_tt(log_2);

end

%% Plot
figure('name', 'Speed Std');
tiledlayout(2,1,'Padding','compact');

% vx
axes(f) = nexttile([1,1]); f=f+1; hold on;
plot(tt.stamp,tt.vx(:,1:tt.max_opp),'Color',col.tt,'DisplayName','tt');
if(compare); plot(tt2.stamp,tt2.vx(:,1:tt2.max_opp),'Color',col.tt2,'DisplayName',name2); end
grid on; title('vx [m/s]'); ylim([0 inf]);

buff_dim = 1.0;
axes(f) = nexttile([1,1]); f=f+1; hold on; hold on;
tt = compute_speed_std(tt);
plot(tt.stamp,tt.vx_std(:,1:tt.max_opp),'Color',col.tt,'DisplayName','tt');
if(compare); tt2 = compute_speed_std(tt2); end
if(compare); plot(tt2.stamp,tt2.vx_std(:,1:tt2.max_opp),'Color',col.tt2,'DisplayName',name2); end
grid on; title('vx std [m/s]'); ylim([0 inf]);

linkaxes(axes, 'x');


%% Functions
function tt = compute_speed_std(tt)

    buff_dim = 1.0;  % trailing window length in seconds
    n = length(tt.stamp);
    nOpp = tt.max_opp;

    tt.vx_std = NaN(n, nOpp);  % preallocate output

    for it = 1:n
        % Define trailing time window
        t_start = tt.stamp(it) - buff_dim;
        t_end   = tt.stamp(it);

        % Find all indices in that time window
        idx = (tt.stamp >= t_start) & (tt.stamp <= t_end);

        % Compute std for each opponent separately
        if any(idx)
            tt.vx_std(it, :) = std(tt.vx(idx, 1:nOpp), 0, 1);
        end
    end
end


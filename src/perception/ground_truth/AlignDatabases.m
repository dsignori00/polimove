clc; close all; clearvars -except log_ref traj_db
%% Paths

proj = currentProject();
addpath(fullfile(proj.RootFolder,'src','perception','utils'));

%load database
if(~exist('traj_db','var'))
    trajDatabase = choose_database();
    if(isempty(trajDatabase))
        error('No database selected');
    else
        tmp = load(trajDatabase);
        traj_db = tmp.trajDatabase;
    end
end

% load ref
if (~exist('log_ref','var'))
[file,path_dir] = uigetfile(fullfile(get_gt_dir(),'*.mat'),'Load ground truth mat');
tmp = load(fullfile(path_dir,file));
fields = fieldnames(tmp);
log_ref = tmp.(fields{1});
clearvars tmp;
end

lap = 3;
lap_idx = log_ref.lap == lap;
x_opp = log_ref.x_map(lap_idx);
y_opp = log_ref.y_map(lap_idx);

% plot track lines
figure("name","Map")
id_left = length(traj_db) - 2;
id_right = length(traj_db) - 1;
hold on
plot(traj_db(id_left).X, traj_db(id_left).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
plot(traj_db(id_right).X, traj_db(id_right).Y, 'color', 'k', 'LineWidth', 1, 'HandleVisibility','off');
plot(x_opp, y_opp,  'color', 'b', 'LineWidth', 1, 'HandleVisibility','off')
axis equal;
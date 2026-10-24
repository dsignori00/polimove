normal_path = get_bags_dir();
opp_dir = get_gt_dir();

% load log
if (~exist('log','var'))
    [file,path] = uigetfile(fullfile(normal_path,'*.mat'),'Load log');
    load(fullfile(path,file), 'log');
end

% load log ref
if  (~exist('log_ref','var'))
    [file,path] = uigetfile(fullfile(opp_dir,'*.mat'),'Load ground truth');
    tmp = load(fullfile(path,file), 'out');
    log_ref = tmp.out;
    clearvars tmp;
end


ego.stamp = log.estimation.stamp__tot;
ego.vx = log.estimation.vx;

gt.stamp = (log_ref.timestamp - double(log.time_offset_nsec))*1e-9;
gt.vx = log_ref.speed;


figure('name', 'Filter - Speed Acc');
hold on;
plot(ego.stamp,ego.vx(:,1),'DisplayName','ego');
plot(gt.stamp,gt.vx,'DisplayName','gt'); 
grid on; title('vx [m/s]');  legend

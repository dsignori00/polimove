clc; clearvars;

bag_folder = get_bags_dir();

if (~exist('log','var'))
    [file,path] = uigetfile(fullfile(bag_folder,'*.mat'),'Load log');
    load(fullfile(path,file), 'log');
end

out.stamp = log.estimation.stamp__tot + log.time_offset_nsec*10^-9;
out.x_cog = log.estimation.x_cog;
out.y_cog = log.estimation.y_cog;
out.vx = log.estimation.vx;
out.ax = log.estimation.ax;
out.heading = log.estimation.heading;

parts = strsplit(file,'_');
T = struct2table(out);
writetable(T, parts{1} + "_polimove_loc.csv", 'Delimiter', ',', 'WriteRowNames',true);

clearvars 
close all
clc

%% Execute

if (~exist('log','var'))
    [file,path] = uigetfile(fullfile(get_bags_dir(),'*.mat'),'Load log');
    load(fullfile(path,file), 'log');
end

log = normalize_stamps(log);

try
    output_path = fullfile(path,file);
    fprintf("Saving processed data to: %s\n", output_path);
    save(output_path, 'log', '-v7.3');
catch e
    warning("WARNING: Could not save processed data");
    warning("Error type:  " + e.message);
end


close all; clearvars; clc;

%#ok<*UNRCH>
%#ok<*INUSD>


folder = "mat/";

%% Process all .mat files in folder
files = dir(fullfile(folder, '*.mat'));

if isempty(files)
    warning("No .mat files found in folder: %s", folder);
    return;
end

for k = 1:numel(files)
    input_file = fullfile(files(k).folder, files(k).name);

    try
        data = load(input_file);

        if ~isfield(data, 'out')
            warning("Skipping %s: variable 'out' not found.", input_file);
            continue;
        end
    
        framelen = floor(data.out.bag_avg_freq) / 2;
        if(mod(framelen,2)==0)
            framelen = framelen + 1;
        end 

        out = data.out;

        out.ax = sgolayfilt(gradient(out.speed(:)) ./ gradient(out.timestamp(:)) * 10^9, 3, framelen);
        out.virtual_acc = true;

        out.yaw_rate = sgolayfilt(gradient(out.yaw_map(:)) ./ gradient(out.timestamp(:)) * 10^9, 3, framelen);
        out.virtual_yawrate = true;

        save(input_file, 'out', '-v7.3');
        fprintf("Updated successfully: %s\n", input_file);
    catch e
        warning("Could not process file: %s", input_file);
        warning("Error type:  " + e.message);
    end
end

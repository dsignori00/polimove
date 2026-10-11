%% Antenna conf check

clearvars -except vectornav nov_bot nov_top
close all

% load opponent mats
if (~exist('vectornav','var'))
    [file,normal_path] = uigetfile('*.mat','Load vectornav mat');
    tmp = load(fullfile(normal_path,file));
    vectornav = tmp.log;
    clearvars tmp;
end

if (~exist('nov_top','var'))
    [file,normal_path] = uigetfile(fullfile(normal_path,'*.mat'),'Load nov_top mat');
    tmp = load(fullfile(normal_path,file));
    nov_top = tmp.log;
    clearvars tmp;
end

if (~exist('nov_bot','var'))
    [file,normal_path] = uigetfile(fullfile(normal_path,'*.mat'),'Load nov_bot mat');
    tmp = load(fullfile(normal_path,file));
    nov_bot = tmp.log;
    clearvars tmp;
end

figure; hold on

for k=1:3

    if(k==1)
        log = vectornav;
    elseif(k==2)
        log=nov_top;
    elseif(k==3)
        log=nov_bot;
    end

    x_map = log.x_map(~isnan(log.x_map));
    y_map = log.y_map(~isnan(log.x_map));
    z_map = log.z_map(~isnan(log.x_map)); 

    if(k==1)
        plot(x_map,y_map,'o','MarkerSize',5,'MarkerFaceColor','k','MarkerEdgeColor','k','DisplayName','vectornav'); 
    elseif(k==2)
        plot(x_map,y_map,'o','MarkerSize',5,'MarkerFaceColor','b','MarkerEdgeColor','b','DisplayName','novatel top');
    elseif(k==3)
        plot(x_map,y_map,'o','MarkerSize',5,'MarkerFaceColor','r','MarkerEdgeColor','r','DisplayName','novatel bot');
    end
    axis equal

end

legend
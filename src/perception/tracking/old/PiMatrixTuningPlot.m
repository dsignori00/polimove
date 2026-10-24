close all
clearvars -except error_matrix

% load log
if (~exist('error_matrix','var'))
    [file,path] = uigetfile('*.mat','Load matrix');
    load(fullfile(path,file), 'error_matrix');
end

tuning_set = [];
colors = ['red' 'blue' 'green' 'black' 'purple'];


%% Plot Error Std

figure('name','PlotTuningPerformance','WindowStyle','normal')
hold on
for i=1:5
    plot(tuning_set, error_matrix(i,:),"Color", colors(i))
end

title('')
xlabel("Element value")
ylabel('Error std')
grid on
legend('x','y','v','a','h')

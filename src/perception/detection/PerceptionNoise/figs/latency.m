%% LATENCY FIGURE
figure('name','Latency', 'NumberTitle', 'off');
tiledlayout(numel(sensors),1,'Padding','compact');

for i = 1:numel(sensors)
    ax(f) = nexttile([1,1]); f=f+1; hold on;
    s = sensors{i}.s;
    plot_detections(s.stamp, s.stamp - s.sens_stamp, 1, sensors{i}.col, sensors{i}.name, '-');
    ylabel(sprintf('%s [s]', sensors{i}.name)); legend show; grid on;
end
xlabel('timestamp [s]');
%% LATENCY FIGURE
figure('name','Latency', 'NumberTitle', 'off');
latency_sources = sensors(cellfun(@(source) source.has_latency,sensors));
latency_layout = tiledlayout(numel(latency_sources),1,'Padding','compact');

for i = 1:numel(latency_sources)
    ax(f) = nexttile(latency_layout); f=f+1; hold on; %#ok<SAGROW>
    s = latency_sources{i}.s;
    plot_detections(s.stamp, s.stamp - s.sens_stamp, 1, ...
        latency_sources{i}.col, latency_sources{i}.name, '-');
    ylabel(sprintf('%s [s]', latency_sources{i}.name)); legend show; grid on;
end
xlabel('timestamp [s]');

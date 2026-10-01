%% LATENCY FIGURE
assert(sum(ACTIVE_CAMERAS)<=1,"VISUALIZE_FRUSTUM must have at most one camera selected")

camera_idx = find(ACTIVE_CAMERAS == true, 1, 'first');
camera_name = CAMERA_NAMES(camera_idx);


figure('name','Reproj - Latency')
tiledlayout(4,1,'Padding','compact');


% LIDAR-CAMERA inference
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    l = sensors{i}.latency;
    if isstruct(l)
        plot_detections(l.(camera_name).stamp, l.(camera_name).delta_lid_cam, 1, sensors{i}.col, sensors{i}.name, '-');
        ylabel("$\Delta_{lid,cam}$", "Interpreter","latex"); legend show; grid on;
    end

end
xlabel('timestamp [s]')

% EST-CAMERA
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    l = sensors{i}.latency;
    if isstruct(l)
        plot_detections(l.(camera_name).stamp, l.(camera_name).delta_est_cam, 1, sensors{i}.col, sensors{i}.name, '-');
        ylabel("$\Delta_{est,cam}$", "Interpreter","latex"); legend show; grid on;
    end
end
xlabel('timestamp [s]')


% MODEL inference
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    l = sensors{i}.latency;
    if isstruct(l)
        plot_detections(l.(camera_name).stamp, l.(camera_name).inference, 1, sensors{i}.col, sensors{i}.name, '-');
        ylabel("inference"); legend show; grid on;
    end
end
xlabel('timestamp [s]');


% Decode pcl
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i = 1:numel(sensors)
    l = sensors{i}.latency;
    if isstruct(l)
        plot_detections(l.(camera_name).stamp, l.(camera_name).delta_lid_cam, 1, sensors{i}.col, sensors{i}.name, '-');
        ylabel("$\Delta_{lid,cam}$", "Interpreter","latex"); legend show; grid on;
    end
end
xlabel('timestamp [s]')
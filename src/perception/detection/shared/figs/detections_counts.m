%% DETECTION COUNT
if exist('detections_parent','var') && isgraphics(detections_parent,'uitab')
    detections_layout_parent = detections_parent;
else
    detections_layout_parent = figure( ...
        'Name','Raw Detections - Count','NumberTitle','off','Color','w');
end
detections_layout = tiledlayout(detections_layout_parent,max(1,numel(sensors)),1, ...
    'Padding','compact','TileSpacing','compact');

detections_first_axis = f;
for i = 1:numel(sensors)
    ax(f) = nexttile(detections_layout,i); %#ok<SAGROW>
    f = f + 1;
    s = sensors{i}.s;

    % Pair every available sensor timestamp with its row's count.
    stamps = double(s.sens_stamp);
    counts = repmat(double(s.count(:)),1,size(stamps,2));
    valid = isfinite(stamps) & isfinite(counts);
    [stamps,idx] = sort(stamps(valid));
    counts = counts(valid);
    plot(stamps,counts(idx),'-o','Color',sensors{i}.col, ...
        'MarkerFaceColor',sensors{i}.col,'MarkerSize',3,'DisplayName',sensors{i}.name);

    ylim([0 s.max_det+1]); grid on;
    ylabel('Detection count');
end

if f > detections_first_axis
    linkaxes(ax(detections_first_axis:f-1),'x');
    xlabel('timestamp [s]');
end

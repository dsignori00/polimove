%% DETECTION COUNT

count_figure = figure('Name','Detection count','NumberTitle','off');
count_layout = tiledlayout(count_figure,1,1,'Padding','compact');

ax(f) = nexttile(count_layout); f = f + 1;
count_axes = ax(f - 1);
hold on;
stairs(rad1.sens_stamp,rad1.count,'Color',col.radar, ...
    'DisplayName',NAME_1 + " - total");
if ground_truth
    stairs(rad1.sens_stamp,rad1.correct_count,'-o', ...
        'Color',col.radar,'MarkerFaceColor',col.radar, ...
        'MarkerSize',sz,'DisplayName',NAME_1 + " - TP");
    stairs(rad1.sens_stamp,rad1.false_positive_count,'--o', ...
        'Color',col.radar,'MarkerFaceColor','none', ...
        'MarkerSize',sz,'DisplayName',NAME_1 + " - FP");
end

if compare
    stairs(rad2_sens_stamp,rad2.count,'Color',col.radar2, ...
        'DisplayName',NAME_2 + " - total");
    if ground_truth
        stairs(rad2_sens_stamp,rad2.correct_count,'-o', ...
            'Color',col.radar2,'MarkerFaceColor',col.radar2, ...
            'MarkerSize',sz,'DisplayName',NAME_2 + " - TP");
        stairs(rad2_sens_stamp,rad2.false_positive_count,'--o', ...
            'Color',col.radar2,'MarkerFaceColor','none', ...
            'MarkerSize',sz,'DisplayName',NAME_2 + " - FP");
    end
    comparison_stats.stamp = rad2_sens_stamp;
    comparison_stats.valid = rad2.valid_detection;
    comparison_stats.correct = rad2.correct;
    comparison_stats.false_positive = rad2.false_positive;
else
    comparison_stats = struct();
end

primary_stats.stamp = rad1.sens_stamp;
primary_stats.valid = rad1.valid_detection;
primary_stats.correct = rad1.correct;
primary_stats.false_positive = rad1.false_positive;

grid on;
xlabel('timestamp [s]');
ylabel('detections [#]');
xlim(x_lim);
legend show;
title('Detection count over time');

if ground_truth
    count_statistics_box = annotation(count_figure,'textbox', ...
        STATISTICS_BOX_POSITION,'Tag','DetectionCountStatistics', ...
        'FitBoxToText','off','BackgroundColor','white', ...
        'EdgeColor','black','Margin',4, ...
        'VerticalAlignment','top','Interpreter','none');
    updateDetectionTotal(count_axes,count_statistics_box,primary_stats, ...
        NAME_1,ground_truth,compare,comparison_stats,NAME_2);
    count_listener = addlistener(count_axes,'XLim','PostSet', ...
        @(~,~) updateDetectionTotal(count_axes,count_statistics_box, ...
        primary_stats,NAME_1,ground_truth,compare,comparison_stats,NAME_2));
    setappdata(count_figure,'DetectionCountXLimListener',count_listener);
end

function updateDetectionTotal(count_axes,statistics_box,primary_stats, ...
        primary_name,ground_truth_enabled,compare_logs,comparison_stats, ...
        comparison_name)
    time_limits = xlim(count_axes);
    statistics_box.String = formatCombinedDetectionStatistics( ...
        primary_stats,primary_name,ground_truth_enabled,compare_logs, ...
        comparison_stats,comparison_name,time_limits);
end

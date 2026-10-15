function combined_text = formatCombinedDetectionStatistics( ...
        primary_stats,primary_name,ground_truth_enabled,compare_logs, ...
        comparison_stats,comparison_name,time_limits)
%FORMATCOMBINEDDETECTIONSTATISTICS Format statistics for one or two logs.

primary_text = formatDetectionStatistics(primary_stats,primary_name, ...
    time_limits,ground_truth_enabled);
if compare_logs
    comparison_text = formatDetectionStatistics(comparison_stats, ...
        comparison_name,time_limits,ground_truth_enabled);
    combined_text = sprintf('%s\n%s',primary_text,comparison_text);
else
    combined_text = primary_text;
end
end

function statistics_text = formatDetectionStatistics(statistics,name, ...
        time_limits,ground_truth_enabled)
selected_window = statistics.stamp >= time_limits(1) & ...
    statistics.stamp <= time_limits(2);
total = nnz(statistics.valid & selected_window);

if ground_truth_enabled
    true_positive = nnz(statistics.correct & selected_window);
    false_positive = nnz(statistics.false_positive & selected_window);
    true_positive_ratio = 100*true_positive/max(total,1);
    false_positive_ratio = 100*false_positive/max(total,1);
    statistics_text = sprintf( ...
        ['%s - Total: %d\n' ...
        'TP: %d (%.1f%%) | FP: %d (%.1f%%)'], ...
        name,total,true_positive,true_positive_ratio,false_positive, ...
        false_positive_ratio);
else
    statistics_text = sprintf('%s - Total: %d',name,total);
end
end

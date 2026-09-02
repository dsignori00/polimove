close all
clearvars -except log log2 trajDatabase gt lid1 lid2

ground_truth = false;
compare = false;

TEMPORAL_GATE = 0.1; % [s]
SPATIAL_GATE = 2.0;  % [m]
STATISTICS_BOX_POSITION = [0.10 0.78 0.28 0.14];

NAME_1 = "LC";
NAME_2 = "LC2";

%#ok<*UNRCH>
%#ok<*INUSD>

%% Paths

proj = currentProject();
addpath(fullfile(proj.RootFolder, 'src', 'perception', 'utils'));

%% Load Data

% Load trajectory database.
if ~exist('trajDatabase', 'var')
    trajDatabase = choose_database();
    if isempty(trajDatabase)
        error('No database selected');
    end
    load(trajDatabase);
end

% Load primary log.
if ~exist('log', 'var')
    [file, path] = uigetfile(fullfile(get_bags_dir(), '*.mat'), 'Load log');
    if isequal(file, 0)
        error('No log selected');
    end
    load(fullfile(path, file));
end

% Load comparison log.
if compare && ~exist('log2', 'var')
    [file, path] = uigetfile(fullfile(get_bags_dir(), '*.mat'), 'Load log 2');
    if isequal(file, 0)
        error('No comparison log selected');
    end
    tmp = load(fullfile(path, file));
    log2 = tmp.log;
end

if ground_truth && ~exist('gt', 'var')
    [file, path] = uigetfile(fullfile(get_gt_dir(), '*.mat'), ...
        'Load ground truth log');
    if isequal(file, 0)
        error('No ground truth log selected');
    end
    tmp = load(fullfile(path, file));
    gt = tmp.out;
end

DateTime = datetime(log.time_offset_nsec, 'ConvertFrom', 'epochtime', ...
    'TicksPerSecond', 1e9, 'Format', 'dd-MMM-yyyy HH:mm:ss');

%% Plot Data

set(0, 'DefaultFigureWindowStyle', 'docked');
set(0, 'DefaultTextInterpreter', 'none');
set(0, 'DefaultLegendInterpreter', 'none');
set(0, 'DefaultLineLineWidth', 2);

x_lim = [0 inf];

%% Naming

col.lidar = '#77AC30';
col.lidar2 = '#A2142F';
col.rho_dot_max = '#EDB120';
col.rho_dot_max2 = '#7E2F8E';
col.ref = '#000000';
sz = 3;
f = 1;

%% Lidar Clustering Detections

lid1 = get_lidar_clustering_fields(log);
if compare
    lid2 = get_lidar_clustering_fields(log2);
end

%% Processing

lid1.range = sqrt(lid1.x_rel.^2 + lid1.y_rel.^2);

if ground_truth
    gt_timestamp = (gt.timestamp - double(log.time_offset_nsec)) * 1e-9;
end

if compare
    lid2.range = sqrt(lid2.x_rel.^2 + lid2.y_rel.^2);
    lid2_sens_stamp = lid2.sens_stamp + ...
        double(log2.time_offset_nsec) * 1e-9 - ...
        double(log.time_offset_nsec) * 1e-9;
    lid2_stamp = lid2.stamp + ...
        double(log2.time_offset_nsec) * 1e-9 - ...
        double(log.time_offset_nsec) * 1e-9;
end

lid1.valid_detection = isfinite(lid1.sens_stamp) & ...
    isfinite(lid1.x_map) & isfinite(lid1.y_map);
lid1.correct = false(size(lid1.valid_detection));
lid1.false_positive = false(size(lid1.valid_detection));

if compare
    lid2.valid_detection = isfinite(lid2_sens_stamp) & ...
        isfinite(lid2.x_map) & isfinite(lid2.y_map);
    lid2.correct = false(size(lid2.valid_detection));
    lid2.false_positive = false(size(lid2.valid_detection));
end

if ground_truth
    [lid1.correct, lid1.false_positive, lid1.valid_detection] = ...
        classify_lidar_detections(lid1.sens_stamp, lid1.x_map, ...
        lid1.y_map, gt_timestamp, gt.x_map, gt.y_map, ...
        TEMPORAL_GATE, SPATIAL_GATE);

    if compare
        [lid2.correct, lid2.false_positive, lid2.valid_detection] = ...
            classify_lidar_detections(lid2_sens_stamp, lid2.x_map, ...
            lid2.y_map, gt_timestamp, gt.x_map, gt.y_map, ...
            TEMPORAL_GATE, SPATIAL_GATE);
    end
end

lid1.correct_count = sum(lid1.correct, 2);
lid1.false_positive_count = sum(lid1.false_positive, 2);
if compare
    lid2.correct_count = sum(lid2.correct, 2);
    lid2.false_positive_count = sum(lid2.false_positive, 2);
end

%% STATE FIGURE COG

figure('Name', 'Detections - CoG')
tiledlayout(2, 1, 'Padding', 'compact');

axesHandles(f) = nexttile([1, 1]); f = f + 1;
hold on;
plotClassifiedDetections(lid1.sens_stamp, lid1.x_rel, lid1.max_det, ...
    col.lidar, NAME_1, sz, ground_truth, lid1.correct, ...
    lid1.false_positive);
if compare
    plotClassifiedDetections(lid2_sens_stamp, lid2.x_rel, lid2.max_det, ...
        col.lidar2, NAME_2, sz, ground_truth, lid2.correct, ...
        lid2.false_positive);
end
if ground_truth
    plot(gt_timestamp, gt.x_rel, 'Color', col.ref, 'DisplayName', 'gt');
end
grid on; title('x rel [m]'); xlim(x_lim); ylim([-200 200]); legend show;

axesHandles(f) = nexttile([1, 1]); f = f + 1;
hold on;
plotClassifiedDetections(lid1.sens_stamp, lid1.y_rel, lid1.max_det, ...
    col.lidar, NAME_1, sz, ground_truth, lid1.correct, ...
    lid1.false_positive);
if compare
    plotClassifiedDetections(lid2_sens_stamp, lid2.y_rel, lid2.max_det, ...
        col.lidar2, NAME_2, sz, ground_truth, lid2.correct, ...
        lid2.false_positive);
end
if ground_truth
    plot(gt_timestamp, gt.y_rel, 'Color', col.ref, 'DisplayName', 'gt');
end
grid on; title('y rel [m]'); xlim(x_lim); ylim([-100 100]); legend show;

%% STATE FIGURE RANGE

figure('Name', 'Detections - Range')
tiledlayout(1, 1, 'Padding', 'compact');

axesHandles(f) = nexttile([1, 1]); f = f + 1;
hold on;
plotClassifiedDetections(lid1.sens_stamp, lid1.range, lid1.max_det, ...
    col.lidar, NAME_1, sz, ground_truth, lid1.correct, ...
    lid1.false_positive);
if compare
    plotClassifiedDetections(lid2_sens_stamp, lid2.range, lid2.max_det, ...
        col.lidar2, NAME_2, sz, ground_truth, lid2.correct, ...
        lid2.false_positive);
end
if ground_truth
    plot(gt_timestamp, gt.rho, 'Color', col.ref, 'DisplayName', 'gt');
end
grid on; title('range [m]'); xlim(x_lim); ylim([0 200]); legend show;

%% STATE FIGURE MAP

figure('Name', 'Detections - Map')
tiledlayout(2, 1, 'Padding', 'compact');

axesHandles(f) = nexttile([1, 1]); f = f + 1;
hold on;
plotClassifiedDetections(lid1.sens_stamp, lid1.x_map, lid1.max_det, ...
    col.lidar, NAME_1, sz, ground_truth, lid1.correct, ...
    lid1.false_positive);
if compare
    plotClassifiedDetections(lid2_sens_stamp, lid2.x_map, lid2.max_det, ...
        col.lidar2, NAME_2, sz, ground_truth, lid2.correct, ...
        lid2.false_positive);
end
if ground_truth
    plot(gt_timestamp, gt.x_map, 'Color', col.ref, 'DisplayName', 'gt');
end
grid on; title('x map [m]'); xlim(x_lim); legend show;

axesHandles(f) = nexttile([1, 1]); f = f + 1;
hold on;
plotClassifiedDetections(lid1.sens_stamp, lid1.y_map, lid1.max_det, ...
    col.lidar, NAME_1, sz, ground_truth, lid1.correct, ...
    lid1.false_positive);
if compare
    plotClassifiedDetections(lid2_sens_stamp, lid2.y_map, lid2.max_det, ...
        col.lidar2, NAME_2, sz, ground_truth, lid2.correct, ...
        lid2.false_positive);
end
if ground_truth
    plot(gt_timestamp, gt.y_map, 'Color', col.ref, 'DisplayName', 'gt');
end
grid on; title('y map [m]'); xlim(x_lim); legend show;

%% DETECTION COUNT FIGURE

countFigure = figure('Name', 'Detections - Count');
tiledlayout(1, 1, 'Padding', 'compact');

axesHandles(f) = nexttile([1, 1]); f = f + 1;
countAxes = axesHandles(f - 1);
hold on;
stairs(lid1.stamp, lid1.count, 'Color', col.lidar, ...
    'DisplayName', NAME_1 + " - total");
if ground_truth
    stairs(lid1.stamp, lid1.correct_count, '-o', 'Color', col.lidar, ...
        'MarkerFaceColor', col.lidar, 'MarkerSize', sz, ...
        'DisplayName', NAME_1 + " - TP");
    stairs(lid1.stamp, lid1.false_positive_count, '--o', ...
        'Color', col.lidar, 'MarkerFaceColor', 'none', ...
        'MarkerSize', sz, 'DisplayName', NAME_1 + " - FP");
end
if compare
    stairs(lid2_stamp, lid2.count, 'Color', col.lidar2, ...
        'DisplayName', NAME_2 + " - total");
    if ground_truth
        stairs(lid2_stamp, lid2.correct_count, '-o', ...
            'Color', col.lidar2, 'MarkerFaceColor', col.lidar2, ...
            'MarkerSize', sz, 'DisplayName', NAME_2 + " - TP");
        stairs(lid2_stamp, lid2.false_positive_count, '--o', ...
            'Color', col.lidar2, 'MarkerFaceColor', 'none', ...
            'MarkerSize', sz, ...
            'DisplayName', NAME_2 + " - FP");
    end
    comparisonStats.stamp = lid2_sens_stamp;
    comparisonStats.valid = lid2.valid_detection;
    comparisonStats.correct = lid2.correct;
    comparisonStats.false_positive = lid2.false_positive;
else
    comparisonStats.stamp = [];
    comparisonStats.valid = [];
    comparisonStats.correct = [];
    comparisonStats.false_positive = [];
end
primaryStats.stamp = lid1.sens_stamp;
primaryStats.valid = lid1.valid_detection;
primaryStats.correct = lid1.correct;
primaryStats.false_positive = lid1.false_positive;
grid on;
xlabel('timestamp [s]');
ylabel('detections [#]');
xlim(x_lim);
legend show;
title('Detection count over time');
countStatisticsBox = annotation(countFigure, 'textbox', ...
    STATISTICS_BOX_POSITION, 'Tag', 'DetectionCountStatistics', ...
    'FitBoxToText', 'off', 'BackgroundColor', 'white', ...
    'EdgeColor', 'black', 'Margin', 4, ...
    'VerticalAlignment', 'top', 'Interpreter', 'none');

% Link every time-series axis. The spatial MAP GUI is intentionally excluded.
linkaxes(axesHandles, 'x');

updateDetectionTotal(countAxes, countStatisticsBox, primaryStats, NAME_1, ...
    ground_truth, compare, comparisonStats, NAME_2);
countListener = addlistener(countAxes, 'XLim', 'PostSet', ...
    @(~, ~) updateDetectionTotal(countAxes, countStatisticsBox, ...
    primaryStats, NAME_1, ground_truth, compare, comparisonStats, NAME_2));
setappdata(countFigure, 'DetectionCountXLimListener', countListener);


%% MAP GUI

mapFigure = figure('Name', 'MAP');
refreshButton = uicontrol(mapFigure, 'Style', 'pushbutton', ...
    'String', 'Refresh', 'Callback', @refreshTimeButtonPushed);

function refreshTimeButtonPushed(src, ~)
    timeAxes = evalin('base', 'axesHandles');
    trajDb = evalin('base', 'trajDatabase');
    groundTruth = evalin('base', 'ground_truth');
    compareLogs = evalin('base', 'compare');
    colors = evalin('base', 'col');
    lidar1 = evalin('base', 'lid1');
    primaryName = evalin('base', 'NAME_1');
    comparisonName = evalin('base', 'NAME_2');
    statisticsBoxPosition = evalin('base', 'STATISTICS_BOX_POSITION');

    if groundTruth
        gtData = evalin('base', 'gt');
        gtTimestamp = evalin('base', 'gt_timestamp');
    end
    if compareLogs
        lidar2 = evalin('base', 'lid2');
        lidar2Timestamp = evalin('base', 'lid2_sens_stamp');
    end

    timeLimits = xlim(timeAxes(1));
    lidar1Idx = lidar1.sens_stamp > timeLimits(1) & ...
        lidar1.sens_stamp < timeLimits(2);
    if compareLogs
        lidar2Idx = lidar2Timestamp > timeLimits(1) & ...
            lidar2Timestamp < timeLimits(2);
    end
    if groundTruth
        gtIdx = gtTimestamp > timeLimits(1) & gtTimestamp < timeLimits(2);
    end

    figureHandle = ancestor(src, 'figure');
    mapAxes = findobj(figureHandle, 'Type', 'axes', ...
        'Tag', 'LidarClusteringMapAxes');
    if isempty(mapAxes)
        mapAxes = axes(figureHandle, 'Tag', 'LidarClusteringMapAxes');
    else
        cla(mapAxes, 'reset');
    end

    hold(mapAxes, 'on');
    grid(mapAxes, 'on');
    title(mapAxes, 'map');
    xlabel(mapAxes, 'x [m]');
    ylabel(mapAxes, 'y [m]');
    axis(mapAxes, 'equal');

    idLeft = length(trajDb) - 2;
    idRight = length(trajDb) - 1;
    plot(mapAxes, trajDb(idLeft).x, trajDb(idLeft).y, ...
        'Color', 'k', 'LineWidth', 1, 'HandleVisibility', 'off');
    plot(mapAxes, trajDb(idRight).x, trajDb(idRight).y, ...
        'Color', 'k', 'LineWidth', 1, 'HandleVisibility', 'off');

    if groundTruth
        lidar1CorrectIdx = lidar1Idx & lidar1.correct;
        lidar1FalsePositiveIdx = lidar1Idx & lidar1.false_positive;
        scatter(mapAxes, lidar1.x_map(lidar1CorrectIdx), ...
            lidar1.y_map(lidar1CorrectIdx), 20, 'filled', ...
            'MarkerEdgeColor', colors.lidar, ...
            'MarkerFaceColor', colors.lidar, ...
            'DisplayName', 'Lidar clustering - TP');
        scatter(mapAxes, lidar1.x_map(lidar1FalsePositiveIdx), ...
            lidar1.y_map(lidar1FalsePositiveIdx), 20, 'o', ...
            'MarkerEdgeColor', colors.lidar, ...
            'MarkerFaceColor', 'none', ...
            'DisplayName', 'Lidar clustering - FP');
    else
        scatter(mapAxes, lidar1.x_map(lidar1Idx), ...
            lidar1.y_map(lidar1Idx), 20, 'filled', ...
            'MarkerEdgeColor', colors.lidar, ...
            'MarkerFaceColor', colors.lidar, ...
            'DisplayName', 'Lidar clustering');
    end

    if compareLogs
        if groundTruth
            lidar2CorrectIdx = lidar2Idx & lidar2.correct;
            lidar2FalsePositiveIdx = lidar2Idx & lidar2.false_positive;
            scatter(mapAxes, lidar2.x_map(lidar2CorrectIdx), ...
                lidar2.y_map(lidar2CorrectIdx), 20, 'filled', ...
                'MarkerEdgeColor', colors.lidar2, ...
                'MarkerFaceColor', colors.lidar2, ...
                'DisplayName', 'Lidar clustering - 2 - TP');
            scatter(mapAxes, lidar2.x_map(lidar2FalsePositiveIdx), ...
                lidar2.y_map(lidar2FalsePositiveIdx), 20, 'o', ...
                'MarkerEdgeColor', colors.lidar2, ...
                'MarkerFaceColor', 'none', ...
                'DisplayName', 'Lidar clustering - 2 - FP');
        else
            scatter(mapAxes, lidar2.x_map(lidar2Idx), ...
                lidar2.y_map(lidar2Idx), 20, 'filled', ...
                'MarkerEdgeColor', colors.lidar2, ...
                'MarkerFaceColor', colors.lidar2, ...
                'DisplayName', 'Lidar clustering - 2');
        end
    end

    if groundTruth
        plot(mapAxes, gtData.x_map(gtIdx), gtData.y_map(gtIdx), ...
            'Color', colors.ref, 'DisplayName', 'Ground truth');
    end

    primaryStats.stamp = lidar1.sens_stamp;
    primaryStats.valid = lidar1.valid_detection;
    primaryStats.correct = lidar1.correct;
    primaryStats.false_positive = lidar1.false_positive;
    if compareLogs
        comparisonStats.stamp = lidar2Timestamp;
        comparisonStats.valid = lidar2.valid_detection;
        comparisonStats.correct = lidar2.correct;
        comparisonStats.false_positive = lidar2.false_positive;
    else
        comparisonStats = struct();
    end

    mapStatisticsBox = findall(figureHandle, ...
        'Tag', 'MapDetectionStatistics');
    if isempty(mapStatisticsBox)
        mapStatisticsBox = annotation(figureHandle, 'textbox', ...
            statisticsBoxPosition, 'Tag', 'MapDetectionStatistics', ...
            'FitBoxToText', 'off', 'BackgroundColor', 'white', ...
            'EdgeColor', 'black', 'Margin', 4, ...
            'VerticalAlignment', 'top', 'Interpreter', 'none');
    end
    mapStatisticsBox.String = formatCombinedDetectionStatistics( ...
        primaryStats, primaryName, groundTruth, compareLogs, ...
        comparisonStats, comparisonName, timeLimits);

    legend(mapAxes, 'show');
end

function plotClassifiedDetections(stamp, values, maxDet, color, name, ...
        markerSize, groundTruthEnabled, correctMask, falsePositiveMask)
    stamp = safe_cols(stamp, maxDet);
    values = safe_cols(values, maxDet);

    if groundTruthEnabled
        correctMask = safe_cols(correctMask, maxDet);
        falsePositiveMask = safe_cols(falsePositiveMask, maxDet);

        correctValues = values;
        correctValues(~correctMask) = NaN;
        falsePositiveValues = values;
        falsePositiveValues(~falsePositiveMask) = NaN;

        plot(stamp, correctValues, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'HandleVisibility', 'off');
        plot(stamp, falsePositiveValues, 'o', ...
            'MarkerFaceColor', 'none', 'MarkerEdgeColor', color, ...
            'MarkerSize', markerSize, 'HandleVisibility', 'off');
        plot(NaN, NaN, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'DisplayName', name + " - TP");
        plot(NaN, NaN, 'o', 'MarkerFaceColor', 'none', ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'DisplayName', name + " - FP");
    else
        plot(stamp, values, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'HandleVisibility', 'off');
        plot(NaN, NaN, 'o', 'MarkerFaceColor', color, ...
            'MarkerEdgeColor', color, 'MarkerSize', markerSize, ...
            'DisplayName', name);
    end
end

function updateDetectionTotal(countAxes, statisticsBox, primaryStats, ...
        primaryName, groundTruthEnabled, compareLogs, comparisonStats, ...
        comparisonName)
    timeLimits = xlim(countAxes);
    statisticsBox.String = formatCombinedDetectionStatistics( ...
        primaryStats, primaryName, groundTruthEnabled, compareLogs, ...
        comparisonStats, comparisonName, timeLimits);
end

function combinedText = formatCombinedDetectionStatistics(primaryStats, ...
        primaryName, groundTruthEnabled, compareLogs, comparisonStats, ...
        comparisonName, timeLimits)
    primaryText = formatDetectionStatistics(primaryStats, primaryName, ...
        timeLimits, groundTruthEnabled);
    if compareLogs
        comparisonText = formatDetectionStatistics(comparisonStats, ...
            comparisonName, timeLimits, groundTruthEnabled);
        combinedText = sprintf('%s\n%s', primaryText, comparisonText);
    else
        combinedText = primaryText;
    end
end

function statisticsText = formatDetectionStatistics(statistics, name, ...
        timeLimits, groundTruthEnabled)
    selectedWindow = statistics.stamp >= timeLimits(1) & ...
        statistics.stamp <= timeLimits(2);
    total = nnz(statistics.valid & selectedWindow);

    if groundTruthEnabled
        correct = nnz(statistics.correct & selectedWindow);
        falsePositive = nnz(statistics.false_positive & selectedWindow);
        truePositiveRatio = 100 * correct / max(total, 1);
        falsePositiveRatio = 100 * falsePositive / max(total, 1);
        statisticsText = sprintf( ...
            ['%s - Total: %d\n' ...
            'TP: %d (%.1f%%) | FP: %d (%.1f%%)'], ...
            name, total, correct, truePositiveRatio, falsePositive, ...
            falsePositiveRatio);
    else
        statisticsText = sprintf('%s - Total: %d', name, total);
    end
end

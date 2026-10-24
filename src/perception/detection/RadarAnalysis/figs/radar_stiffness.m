%% RADAR SUPPORT STIFFNESS
% Linear spring proxy:
%   angular_error [deg] = offset [deg] + compliance [deg/g] * ay [g]
%
% The compliance slope quantifies support deflection per lateral load, and
% K* = 1/abs(compliance) is a normalized stiffness proxy. A physical
% stiffness requires the support mass/inertia and moment arm.

STIFFNESS.MIN_RANGE = 15.0;          % Avoid ill-conditioned bearing errors [m].
STIFFNESS.MAX_TIME_GAP = 0.01;      % Maximum GT/ego interpolation gap [s].
STIFFNESS.MIN_SAMPLES = 3;
STIFFNESS.GRAVITY = 9.80665;        % Standard gravity [m/s^2].


if ~ground_truth
    warning('RadarAnalysis:StiffnessRequiresGroundTruth', ...
        ['The stiffness figure requires ground truth. Set ' ...
        'ground_truth = true to generate it.']);
end

primary_data = prepareRadarStiffnessData( ...
    rad1,rad1.sens_stamp,rad1.sens_stamp, ...
    log.estimation.stamp__tot,log.estimation.ay, ...
    gt_timestamp,gt.x_rel,gt.y_rel,rad1.correct,STIFFNESS);

stiffness_datasets = struct( ...
    'name',NAME_1, ...
    'color',col.radar, ...
    'data',primary_data);

if compare
    comparison_data = prepareRadarStiffnessData( ...
        rad2,rad2_sens_stamp,rad2.sens_stamp, ...
        log2.estimation.stamp__tot,log2.estimation.ay, ...
        gt_timestamp,gt.x_rel,gt.y_rel,rad2.correct,STIFFNESS);
    stiffness_datasets(2) = struct( ...
        'name',NAME_2, ...
        'color',col.radar2, ...
        'data',comparison_data);
end

stiffness_figure = figure( ...
    'Name','Radar support stiffness', ...
    'NumberTitle','off', ...
    'Color','w');
stiffness_tabs = uitabgroup(stiffness_figure);

direction_titles = ["Front","Back"];
direction_fields = ["front","back"];
direction_conditions = ["x_{rel}>0","x_{rel}<0"];
number_of_results = numel(direction_fields)*numel(stiffness_datasets);

result_direction = strings(number_of_results,1);
result_dataset = strings(number_of_results,1);
result_samples = zeros(number_of_results,1);
result_compliance = nan(number_of_results,1);
result_stiffness_proxy = nan(number_of_results,1);
result_offset = nan(number_of_results,1);
result_r_squared = nan(number_of_results,1);
result_r = nan(number_of_results,1);
result_p_value = nan(number_of_results,1);
result_rmse = nan(number_of_results,1);
result_index = 0;

for direction_index = 1:numel(direction_fields)
    direction = direction_fields(direction_index);
    direction_tab = uitab(stiffness_tabs, ...
        'Title',direction_titles(direction_index));
    direction_layout = tiledlayout(direction_tab,2,1, ...
        'Padding','compact','TileSpacing','compact');

    scatter_axes = nexttile(direction_layout);
    hold(scatter_axes,'on');
    grid(scatter_axes,'on');
    box(scatter_axes,'on');
    xline(scatter_axes,0,'--k','HandleVisibility','off');
    yline(scatter_axes,0,'--k','HandleVisibility','off');
    ylabel(scatter_axes,'angular error [deg]');
    title(scatter_axes, ...
        'Radar bearing error versus ego lateral acceleration');

    residual_axes = nexttile(direction_layout);
    hold(residual_axes,'on');
    grid(residual_axes,'on');
    box(residual_axes,'on');
    xline(residual_axes,0,'--k','HandleVisibility','off');
    yline(residual_axes,0,'--k','HandleVisibility','off');
    xlabel(residual_axes,'ego lateral acceleration [g]');
    ylabel(residual_axes,'fit residual [deg]');
    title(residual_axes,'Linear-model residual');

    for dataset_index = 1:numel(stiffness_datasets)
        dataset = stiffness_datasets(dataset_index);
        selected = dataset.data.(direction);
        lateral_acceleration = dataset.data.lateral_acceleration_g(selected);
        angular_error = dataset.data.angular_error_deg(selected);

        model = fitRadarStiffnessModel( ...
            lateral_acceleration,angular_error,STIFFNESS.MIN_SAMPLES);

        if isscalar(stiffness_datasets)
            measurement_label = 'Measurement error';
            fit_prefix = 'Fit';
        else
            measurement_label = sprintf('%s measurement error',dataset.name);
            fit_prefix = sprintf('%s fit',dataset.name);
        end

        plot(scatter_axes,lateral_acceleration,angular_error,'.', ...
            'Color',dataset.color,'MarkerSize',7, ...
            'DisplayName',measurement_label);

        if model.valid
            fit_acceleration = linspace( ...
                min(lateral_acceleration),max(lateral_acceleration),100);
            fit_label = sprintf([ ...
                '%s ($%s$): ' ...
                '$\\theta_0 = %+.3f^{\\circ},\\;C_{\\theta} = ' ...
                '%+.4f^{\\circ}/g,\\;R^2 = %.3f$'], ...
                fit_prefix,direction_conditions(direction_index), ...
                model.offset_deg,model.compliance_deg_per_g, ...
                model.r_squared);
            plot(scatter_axes,fit_acceleration, ...
                polyval(model.coefficients,fit_acceleration),'-', ...
                'Color',dataset.color,'LineWidth',2, ...
                'DisplayName',fit_label);
            plot(residual_axes,lateral_acceleration,model.residuals,'.', ...
                'Color',dataset.color,'MarkerSize',7, ...
                'DisplayName',dataset.name);
        end

        result_index = result_index + 1;
        result_direction(result_index) = direction_titles(direction_index);
        result_dataset(result_index) = dataset.name;
        result_samples(result_index) = model.samples;
        result_compliance(result_index) = model.compliance_deg_per_g;
        result_stiffness_proxy(result_index) = ...
            model.stiffness_proxy_g_per_deg;
        result_offset(result_index) = model.offset_deg;
        result_r_squared(result_index) = model.r_squared;
        result_r(result_index) = model.correlation;
        result_p_value(result_index) = model.p_value;
        result_rmse(result_index) = model.rmse_deg;
    end

    legend(scatter_axes,'show','Location','northwest','Interpreter','latex');
    legend(residual_axes,'show','Location','northwest');
end

stiffness_results = table( ...
    result_direction,result_dataset,result_samples,result_compliance, ...
    result_stiffness_proxy,result_offset,result_r_squared,result_r, ...
    result_p_value,result_rmse, ...
    'VariableNames',{ ...
    'Direction','Dataset','Samples','ComplianceDegPerG', ...
    'StiffnessProxyGPerDeg','OffsetDeg','RSquared','Correlation', ...
    'PValue','RmseDeg'});

disp('Radar support stiffness linear-fit results:');
disp(stiffness_results);

function model = fitRadarStiffnessModel( ...
        lateral_acceleration_g,angular_error_deg,min_samples)
%FITRADARSTIFFNESSMODEL Fit angular error as a linear function of lateral load.

lateral_acceleration_g = double(lateral_acceleration_g(:));
angular_error_deg = double(angular_error_deg(:));
valid = isfinite(lateral_acceleration_g) & isfinite(angular_error_deg);
lateral_acceleration_g = lateral_acceleration_g(valid);
angular_error_deg = angular_error_deg(valid);

model.valid = false;
model.samples = numel(lateral_acceleration_g);
model.coefficients = [NaN NaN];
model.compliance_deg_per_g = NaN;
model.stiffness_proxy_g_per_deg = NaN;
model.offset_deg = NaN;
model.r_squared = NaN;
model.correlation = NaN;
model.p_value = NaN;
model.rmse_deg = NaN;
model.residuals = nan(size(angular_error_deg));

if model.samples < min_samples || ...
        max(lateral_acceleration_g) - min(lateral_acceleration_g) <= sqrt(eps)
    return;
end

model.coefficients = polyfit(lateral_acceleration_g,angular_error_deg,1);
prediction = polyval(model.coefficients,lateral_acceleration_g);
model.residuals = angular_error_deg - prediction;

sum_squared_residuals = sum(model.residuals.^2);
sum_squared_total = sum( ...
    (angular_error_deg - mean(angular_error_deg)).^2);
if sum_squared_total > 0
    model.r_squared = 1 - sum_squared_residuals/sum_squared_total;
end

[correlation_matrix,p_value_matrix] = corrcoef( ...
    lateral_acceleration_g,angular_error_deg);
if isequal(size(correlation_matrix),[2 2])
    model.correlation = correlation_matrix(1,2);
    model.p_value = p_value_matrix(1,2);
end

model.compliance_deg_per_g = model.coefficients(1);
if abs(model.compliance_deg_per_g) > eps
    model.stiffness_proxy_g_per_deg = 1/abs(model.compliance_deg_per_g);
end
model.offset_deg = model.coefficients(2);
model.rmse_deg = sqrt(mean(model.residuals.^2));
model.valid = true;
end

%% CORRELATION

% Remember that:
% p-value: The probability of obtaining test results at least as extreme as
%          the result actually observed, under the assumption that the null
%          hypothesis is correct
%
% r coeff: The correlation coefficient is the specific measure that 
%          quantifies the strength of the linear relationship between 
%          two variables in a correlation analysis. Positive r values
%          indicate a positive correlation, where the values of both 
%          variables tend to increase together.

corr_value = abs(log_ref.rho);
corr_name  = 'range [m]';
corr_y_lim = [0 100]; 

%% CORRELATION
corr_stamp = gt.stamp;

for k = 1:numel(sensors)
    sensors{k}.P_x = nan(2,2);
    sensors{k}.R_x = nan(2,2);
    sensors{k}.P_y = nan(2,2);
    sensors{k}.R_y = nan(2,2);
end

% Plotting
figure('Name','Correlations')
tiledlayout(3,1,'Padding','compact');

% 1) Reference signal
ax(f)=nexttile; hold on; grid on; f = f+1;
plot(corr_stamp, corr_value, 'DisplayName', corr_name)
title("Searching correlation with: " + corr_name)
ylabel(corr_name); ylim(corr_y_lim);
legend

% 2) X rel errors
ax(f)=nexttile; hold on; grid on; x_ax = ax(f); f = f+1; 
yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')
for k = 1:length(sensors)
    s = sensors{k};
    p_str = sprintf('%.2f', s.R_x(2,1));
    plot_detections(s.s.sens_stamp, s.s.x_rel_err, sensors{k}.s.max_det, sensors{k}.col, ['r=' p_str ' - ' s.name ]);
end
ylim(y_err_lim); legend show; ylabel('x rel error [m]');

% 3) Y rel errors
ax(f)=nexttile; hold on; grid on; y_ax = ax(f); f = f+1;
yline(0,'--k','LineWidth',0.3,'HandleVisibility','off')
for k = 1:length(sensors)
    s = sensors{k};
    p_str = sprintf('%.2f', s.R_y(2,1));
    plot_detections(s.s.sens_stamp, s.s.y_rel_err, sensors{k}.s.max_det, sensors{k}.col, ['r=' p_str ' - ' s.name ]);
end
ylim(y_err_lim); legend show; ylabel('y rel error [m]'); xlabel('timestamp [s]');


%% UPDATE LEGENDS ON TIME LIMIT CHANGE

ax_time = ax(1);   % axis that controls the time window
addlistener(ax_time, 'XLim', 'PostSet', @onXLimChanged);
onXLimChanged([],[])

function onXLimChanged(~,~)

    sensors     = evalin('base','sensors');
    ax          = evalin('base','ax');
    corr_stamp  = evalin('base','corr_stamp');
    corr_value  = evalin('base','corr_value');
    err_thr     = evalin('base','err_thr');

    t_lim = xlim(ax(1));

    for k = 1:numel(sensors)
        s = sensors{k}.s;

        % --- time window ---
        [t1, tend] = timeWindowIdx(s.sens_stamp, t_lim);
        time = false(size(s.sens_stamp));
        time(t1:tend) = true;

        ass = hypot(s.x_map_err, s.y_map_err) < err_thr;
        idx = ass & repmat(time,1,size(s.x_map_err,2));

        [R_x,P_x,R_y,P_y] = corr_with_ref(s,idx,corr_stamp,corr_value);

        sensors{k}.R_x = R_x;
        sensors{k}.P_x = P_x;
        sensors{k}.R_y = R_y;
        sensors{k}.P_y = P_y;
    end

    assignin('base','sensors',sensors)
    updateLegends(sensors)
end


function updateLegends(sensors)
    x_ax = evalin('base','x_ax');
    y_ax = evalin('base','y_ax');

    % --- X errors legend ---
    lgd_x = legend(x_ax);
    strs = strings(1,numel(sensors));

    for k = 1:numel(sensors)
        strs(k) = sprintf('r=%.2f - %s', ...
            sensors{k}.R_x(2,1), sensors{k}.name);
    end
    lgd_x.String = strs;

    % --- Y errors legend ---
    lgd_y = legend(y_ax);
    strs = strings(1,numel(sensors));

    for k = 1:numel(sensors)
        strs(k) = sprintf('r=%.2f - %s', ...
            sensors{k}.R_y(2,1), sensors{k}.name);
    end
    lgd_y.String = strs;
end

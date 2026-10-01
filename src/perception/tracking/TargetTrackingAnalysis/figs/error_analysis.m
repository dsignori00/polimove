if(~use_ref && ~use_sim_ref)
   print("[ErrorAnalysis] no ground truth given")
   return
end

% Error Analysis 
figure("Name","Tracking Error", 'NumberTitle','off');

% button
c = c + 1;
b(c) = uicontrol('Style','pushbutton', ...
    'String','Refresh', ...
    'Units','normalized', ...
    'Position',[0.01 0.01 0.1 0.05], ...
    'Callback',@refreshErrorSummaryButtonPushed);

function refreshErrorSummaryButtonPushed(~,~)

    % --- fetch from base ---
    tt       = evalin('base','tt');
    ax_ref   = evalin('base','ax');
    err_thr  = evalin('base','err_thr');
    errors   = evalin('base','errors');
    compare  = evalin('base','compare');
    compare2 = evalin('base','compare2');
    stats    = evalin('base','err_stats');

    if compare
        tt2     = evalin('base','tt2');
        errors2  = evalin('base','errors2');
    end
    if compare2
        tt3     = evalin('base','tt3');
        errors3  = evalin('base','errors3');
    end

    % --- compute stats for specified time window ---

    % time window from reference axis
    t_lim = xlim(ax_ref(1));
    [t1, tend] = timeWindowIdx(tt.stamp, t_lim);
    time = false(size(tt.stamp));
    time(t1:tend) = true;
    ass = hypot(errors.x_map_err, errors.y_map_err) < err_thr;
    errors.idx = ass & time;

    if(compare)
        [t1_2, tend_2] = timeWindowIdx(tt2.stamp, t_lim);
        time = false(size(tt2.stamp));
        time(t1_2:tend_2) = true;
        ass = hypot(errors2.x_map_err, errors2.y_map_err) < err_thr;
        errors2.idx = ass & time;
    end
    if(compare2)
        [t1_3, tend_3] = timeWindowIdx(tt3.stamp, t_lim);
        time = false(size(tt3.stamp));
        time(t1_3:tend_3) = true;
        ass = hypot(errors3.x_map_err, errors3.y_map_err) < err_thr;
        errors3.idx = ass & time;
    end

    for l = stats
        err = errors.([l{1} '_err']);
        errors.([l{1} '_std'])  = std(err(errors.idx));
        errors.([l{1} '_mean']) = mean(err(errors.idx));
        if(compare)
            err2 = errors2.([l{1} '_err']);
            errors2.([l{1} '_std'])  = std(err2(errors2.idx));
            errors2.([l{1} '_mean']) = mean(err2(errors2.idx));
        end
        if(compare2)
            err3 = errors3.([l{1} '_err']);
            errors3.([l{1} '_std'])  = std(err3(errors3.idx));
            errors3.([l{1} '_mean']) = mean(err3(errors3.idx));
        end
    end

    fprintf('\n');
    fprintf('====================================================================================================\n');
    
    % Header
    fprintf('%-15s | ', 'Algorithm');
    for i = 1:numel(stats)
        fprintf('%-25s', stats{i});
    end
    fprintf('\n');
    fprintf('====================================================================================================\n');
    
    % -------- BASE --------
    fprintf('%-15s | ', tt.name);
    for i = 1:numel(stats)
        name = stats{i};
        m = errors.([name '_mean']);
        s = errors.([name '_std']);
        fprintf('%-25s', sprintf('%.6f ± %.6f', m, s));
    end
    fprintf('\n');
    
    % -------- COMPARE 1 --------
    if compare
        fprintf('%-15s | ', tt2.name);
        for i = 1:numel(stats)
            name = stats{i};
            m = errors2.([name '_mean']);
            s = errors2.([name '_std']);
            fprintf('%-25s', sprintf('%.6f ± %.6f', m, s));
        end
        fprintf('\n');
    end
    
    % -------- COMPARE 2 --------
    if compare2
        fprintf('%-15s | ', tt3.name);
        for i = 1:numel(stats)
            name = stats{i};
            m = errors3.([name '_mean']);
            s = errors3.([name '_std']);
            fprintf('%-25s', sprintf('%.6f ± %.6f', m, s));
        end
        fprintf('\n');
    end
    
    fprintf('====================================================================================================\n');

    [errors.x_ellipse, errors.y_ellipse] = calculate_ellipse( ...
        errors.x_rel_std, errors.y_rel_std, ...
        errors.x_rel_mean, errors.y_rel_mean);
    
    if compare 
        [errors2.x_ellipse, errors2.y_ellipse] = calculate_ellipse( ...
            errors2.x_rel_std, errors2.y_rel_std, ...
            errors2.x_rel_mean, errors2.y_rel_mean);
    end
    if compare2
        [errors3.x_ellipse, errors3.y_ellipse] = calculate_ellipse( ...
            errors3.x_rel_std, errors3.y_rel_std, ...
            errors3.x_rel_mean, errors3.y_rel_mean);
    end

    % --- Positional errors (ellipse) ---
    n_rows = numel(stats);
    ax_pos = subplot(n_rows,2,1:2:n_rows*2-1);
    cla reset; hold(ax_pos,'on'); grid(ax_pos,'on'); axis(ax_pos,'equal')

    plot(ax_pos, errors.y_ellipse, errors.x_ellipse, ...
        'Color', tt.col, ...
        'LineWidth', 2, ...
        'DisplayName', tt.name)

    scatter(ax_pos, errors.y_rel_mean, errors.x_rel_mean, ...
        'o', ...
        'MarkerFaceColor', tt.col, ...
        'MarkerEdgeColor', tt.col, ...
        'HandleVisibility','off')
    
    str = sprintf('σ_x = %.2f m\nσ_y = %.2f m', ...
        errors.x_rel_std, errors.y_rel_std);
    
    plot_text(ax_pos, [0.3, 0.92], str, tt.col);

    if compare
        plot(ax_pos, errors2.y_ellipse, errors2.x_ellipse, ...
            'Color', tt2.col, ...
            'LineWidth', 2, ...
            'DisplayName', tt2.name)

        scatter(ax_pos, errors2.y_rel_mean, errors2.x_rel_mean, ...
            'o', ...
            'MarkerFaceColor', tt2.col, ...
            'MarkerEdgeColor', tt2.col, ...
            'HandleVisibility','off')
        
        str2 = sprintf('σ_x = %.2f m\nσ_y = %.2f m', ...
            errors2.x_rel_std, errors2.y_rel_std);
        plot_text(ax_pos, [0.3, 0.84], str2, tt2.col);

    end
    if compare2
        plot(ax_pos, errors3.y_ellipse, errors3.x_ellipse, ...
            'Color', tt3.col, ...
            'LineWidth', 2, ...
            'DisplayName', tt3.name)

        scatter(ax_pos, errors3.y_rel_mean, errors3.x_rel_mean, ...
            'o', ...
            'MarkerFaceColor', tt3.col, ...
            'MarkerEdgeColor', tt3.col, ...
            'HandleVisibility','off')

        str3 = sprintf('σ_x = %.2f m\nσ_y = %.2f m', ...
            errors3.x_rel_std, errors3.y_rel_std);
        plot_text(ax_pos, [0.3, 0.76], str3, tt3.col);

    end

    xline(ax_pos,0,'--','LineWidth',0.3,'HandleVisibility','off')
    yline(ax_pos,0,'--','LineWidth',0.3,'HandleVisibility','off')
    title(ax_pos,'Positional Errors')
    xlabel(ax_pos,'y rel [m]')
    ylabel(ax_pos,'x rel [m]')
    legend(ax_pos,'show')

    % --- Error distribution ---
    ax_hist = gobjects(1,numel(stats)); k = 0;
    for l = stats
        k = k + 1;
        ax_hist(k) = subplot(n_rows,2,k*2);
        cla reset; hold(ax_hist(k),'on'); grid(ax_hist(k),'on')

        maxVal = 3*errors.([l{1} '_std']);
        
        histogram(ax_hist(k), errors.([l{1} '_err'])(errors.idx), ...
                'Normalization','pdf', ...
                'FaceColor', tt.col, ...
                'DisplayName', tt.name);

        if compare
            histogram(ax_hist(k), errors2.([l{1} '_err'])(errors2.idx), ...
                'Normalization','pdf', ...
                'FaceColor', tt2.col, ...
                'DisplayName', tt2.name);
            maxVal = [maxVal, 3*errors2.([l{1} '_std'])];
        end

        if compare2
            histogram(ax_hist(k), errors3.([l{1} '_err'])(errors3.idx), ...
                'Normalization','pdf', ...
                'FaceColor', tt3.col, ...
                'DisplayName', tt3.name);
            maxVal = [maxVal, 3*errors3.([l{1} '_std'])];
        end

        % Center x-axis at 0
        maxVal = max(maxVal(:), [], 'omitnan');
        xlim(ax_hist(k), [-maxVal, maxVal]);

        legend(ax_hist(k), ...
            'Interpreter','none', ...
            'Box','on');

        ylbl = sprintf(l{1});
        ylbl = strrep(ylbl, '_', ' ');
        ylabel(ax_hist(k), ylbl)
    end
    title(ax_hist(1),'Error Distribution - PDF')

end

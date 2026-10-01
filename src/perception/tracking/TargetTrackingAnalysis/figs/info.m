%% INFO
figure('Name','Info', 'NumberTitle', 'off');

if(isfield(log,'planner_manager'))
    % racetype
    tiledlayout(2,1,'Padding','compact');
    ax(f) = nexttile([1,1]); f=f+1; hold on;
    plot(log.planner_manager.stamp__tot, log.planner_manager.race_type,'Color',col.tt);
    ylim([-1 5]); grid on; ylabel('RaceType');
else 
    tiledlayout(1,1,'Padding','compact');
end

% decision maker
ax(f) = nexttile([1,1]); f=f+1; hold on;
plot(log.decision_maker.stamp__tot, log.decision_maker.current_state__type, 'Color',col.tt, 'HandleVisibility','off');
yticks(0:4);
ylim([-1 5]); grid on; ylabel('Decision Maker state');
labels = {'0 - RACING','1 - TAILGATING','2 - OVERTAKE','3 - ABORT','4 - CRITICAL'};
for i = 1:numel(labels)
    plot(nan, nan, 'DisplayName', labels{i}, 'Color', 'none');
end

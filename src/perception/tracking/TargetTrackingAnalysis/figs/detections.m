%% RANGE
figure('name', 'Detections - Count', 'NumberTitle', 'off');
tiledlayout(1,1,'Padding','compact');

% % count
% ax(f) = nexttile([1,1]); f=f+1; hold on;
% for i = 1:numel(sensors)
%     s = sensors{i}.s;
%     if isfield(s, 'count') && any(isfinite(s.count(:))) && any(isfinite(s.sens_stamp(:)))
%         scatter(s.sens_stamp, s.count, 36, sensors{i}.col,'filled', 'DisplayName', sensors{i}.name);
%     end
% end
% grid on; ylabel('Count'); legend show; xlabel('timestamp [s]');
% 
% 
% %% GLOBAL OOSM ANALYSIS (multi-sensor)
% 
% allStamp     = [];
% allSensStamp = [];
% allSensorId  = [];
% 
% for i = 1:numel(sensors)
%     s = sensors{i}.s;
%     valid = isfinite(s.stamp(:)) & isfinite(s.sens_stamp(:));
%     if ~any(valid)
%         continue;
%     end
% 
%     allStamp     = [allStamp;     s.stamp(valid)];
%     allSensStamp = [allSensStamp; s.sens_stamp(valid)];
%     allSensorId  = [allSensorId;  repmat(i,sum(valid),1)];
% end
% 
% % Sort by arrival time (filter iteration order)
% [allStamp, ord] = sort(allStamp);
% allSensStamp    = allSensStamp(ord);
% allSensorId     = allSensorId(ord);
% 
% % Detect reprocessed (OOSM) measurements
% N = numel(allStamp);
% isReprocessed = false(N,1);
% 
% maxSensStamp = -inf;
% for k = 1:N
%     if allSensStamp(k) < maxSensStamp
%         isReprocessed(k) = true;
%     else
%         maxSensStamp = allSensStamp(k);
%     end
% end
% 
% % Count OOSM per filter iteration
% [iterStamp, ~, iterIdx] = unique(allStamp);
% numReprocessedPerIter = accumarray(iterIdx, isReprocessed);
% 
% % latency figure with OOSM overlay
% ax(f) = nexttile([1,1]);  f = f + 1; hold on;
% stairs(iterStamp, numReprocessedPerIter, 'Color', col.tt, 'DisplayName','OOSM count');
% ylabel('reprocessed measures [-]'); xlabel('timestamp [s]');

% opponent count
ax(f) = nexttile([1,1]); f=f+1; hold on;
plot_tt(tt.stamp, tt.count, 1, col.tt, name1);
if(compare); plot_tt(tt2.stamp, tt2.count, 1, col.tt2, name2); end
if(compare2); plot_tt(tt3.stamp, tt3.count, 1, col.tt3, name3); end
grid on; ylabel('Opponent count'); xlabel('timestamp [s]'); legend show;

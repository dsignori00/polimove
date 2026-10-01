%% FIGURE IMM

if ~isfield(tt,'imm') 
    return;
end

figure('name','Imm', 'NumberTitle', 'off');
tiledlayout(3,1,'Padding','compact');

if(~exist('use_sim_ref','var'))
    use_sim_ref = false;
end

% IMM model probabilities
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i=1:size(tt.imm.probabilities,3)
    plot(tt.stamp,tt.imm.probabilities(:,1,i),'Color',colors.matlab{i},'DisplayName',sprintf('Model %d', i));
end
grid on; ylabel('model probs [\%]'); legend

% IMM model likelihoods
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i=1:size(tt.imm.likelihoods,3)
    plot(tt.stamp,tt.imm.likelihoods(:,1,i),'Color',colors.matlab{i},'DisplayName',sprintf('Model %d', i));
end
grid on; ylabel('model likelihoods'); legend

% IMM model residuals
ax(f) = nexttile([1,1]); f=f+1; hold on;
for i=1:size(tt.imm.residuals,3)
    plot(tt.stamp,tt.imm.residuals(:,1,i),'Color',colors.matlab{i},'DisplayName',sprintf('Model %d', i));
end
grid on; ylabel('model residuals'); legend

xlabel("Time [s]");

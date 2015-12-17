load sync_control.log;
load sync_estimate.log;  
load sync_fcs.log;       
load sync_solution.log;

% Sync resolution
res = 0.001;

% Get the min and max times
mint = min([sync_estimate(1,1)   sync_fcs(1,1)   sync_solution(1,1)]);
maxt = max([sync_estimate(end,1) sync_fcs(end,1) sync_solution(end,1)]);
tick = mint:res:maxt;

est = interp1(  sync_estimate(:,1),...  %time
                sync_estimate(:,2),...  %signal
                tick,...                %sample
                'linear',0);     %technique 
sol = interp1(  sync_solution(:,1),...  %time
                sync_solution(:,5),...  %signal
                tick,...                %sample
                'linear',0);     %technique
fcs = interp1(  sync_fcs(2:end,1),...       %time
                sync_fcs(2:end,5),...       %signal
                tick,...                %sample
                'linear',0);     %technique

% Normalize signals to zero mean and unit variance
%sol = (sol - mean(sol)) / std(sol);
%est = (est - mean(est)) / std(est);
%fcs = (fcs - mean(fcs)) / std(fcs);

% Compute the lag between est and fcs
[xc,lags] = xcorr(fcs, est);
[~,I] = max(abs(xc));
sync_fcs(:,1) = sync_fcs(:,1) - lags(I)*res;
lags(I)*res

% Compute time lag between est and solution
[xc,lags] = xcorr(sol, est);
[~,I] = max(abs(xc));
sync_solution(:,1) = sync_solution(:,1) - lags(I)*res;
lags(I)*res

% Plot 
figure; hold on; grid on; ylim([-20 20]);
h1 = plot(sync_estimate(:,1),sync_estimate(:,2),'r-');
h2 = plot(sync_solution(:,1),sync_solution(:,5),'g-');
h3 = plot(sync_fcs(:,1),sync_fcs(:,5),'b-');
title('Synchronised streams');
legend([h1 h2 h3],{'Estimate','Solution','FCS'});
xlabel('Time (seconds)');
ylabel('X position');


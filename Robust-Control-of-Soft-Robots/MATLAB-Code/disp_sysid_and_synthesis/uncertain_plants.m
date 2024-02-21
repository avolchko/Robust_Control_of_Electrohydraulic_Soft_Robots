% develop a slew of plants for uncertainty analysis
% using nonlinearities available in sensing mapping
clear  g_err
s = tf('s');


%% Multiplicative Uncertainty
gnom = sys_selc
f2 = figure(2);
for m = 1:length(gp) 
    % multiplicative input uncertainty
    %g_err{m} = gnom\(gp{m} - gnom );
    
    % multiplicative output uncertainty
    % g_err{m} = (gp{m} - gnom )/gnom;

    % additive uncertainty
    g_err{m} = (gp{m} - gnom); 
    h = sigmaplot(g_err{m});
    hold on
end
    
legend 
%
% selecting an uncertainty weight
% create weighting function to bound this 
% use form 
min = 0.65;
max = 0.03;
T = 0.036;
s = tf('s');
WI1 = ((T*s + min) / (((T/max)* s + 1)));         % Sensitivity weight
min = 0.35;
max = 0.01;
T = 0.00026;
WI2 = ((T*s + min) / (((T/max)* s + 1)));  

WI = WI1 + WI2;

h = sigmaplot(WI, '--k');


setoptions(h,'FreqUnits','Hz', 'MagUnits' , 'abs', 'Grid','on');
% xlim([10e-2 300])

set(findall(gcf,'type','line'),'linewidth',1.2);

title({'Modeled Uncertainty Singular Values', 'Displacement Dynamics'})
xlim([2e-2 1e2]);

legend off
f2.Position = [100 100 350 300];
%%
f9 = figure(9);
L = gnom * K;
S = inv(eye(2)+L);                   % Sensitivity 
T = eye(2)-S;                      % Complementary sensitivity 

h = sigmaplot(WI1*T, 'k');
ylim([0 1])
xlim([2e-2 1e2])
grid on

f9.Position = [100 100 350 300];
title({'Weighted Complementary Sensitivity', 'Displacement Dynamics'})
setoptions(h,'FreqUnits','Hz', 'MagUnits' , 'abs', 'Grid','on');
set(findall(gcf,'type','line'),'linewidth',1.2);
xlim([2e-2 1e2]);

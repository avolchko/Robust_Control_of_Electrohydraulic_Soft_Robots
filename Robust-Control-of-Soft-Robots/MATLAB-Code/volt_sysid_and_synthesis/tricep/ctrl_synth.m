%% loop shaping
s = tf('s')
% was db = 10, wc = 200*2*pi wo = wc/5 k_lag = 5* alpha*(tau*s + 1 ) / (alpha * tau * s + 1 );
s = tf('s');
f5 = figure(5);
r = bodeplot(g_nom);

% choose alpha to fit tracking requirement
db = 15;
alpha = db2mag(db);
% wc is system bandwidth
wc = 2000*2*pi;
% choose break freq, wo, to be a decade below wc
wo = wc / 10 ;
tau = 1/wo;
% compute 1/ alpha tau
wo_p = 1/(alpha*tau);

% create lag controller 

k_lag = alpha*(tau*s + 1 ) / (alpha * tau * s + 1 );

hold on

bodeplot(k_lag);

hold on

h = bodeplot(g_nom*k_lag);
[gm,pm] = margin(g_nom*k_lag)

% 
setoptions(r, 'FreqUnits', 'Hz', 'MagUnits', 'dB');
% setoptions(r, )
xlim([0.1 200])
% legend('plant', 'lag controller','lag loop', 'pid controller', 'pid loop')
grid minor
set(findall(gcf,'type','line'),'linewidth',1.2);
f5.Position = [100 100 350 300];

ts = 1/1000;
Cd = c2d(k_lag, ts)
Cd.Variable = 'z^-1' 
gd = c2d(g_nom, ts);

% % phase margin
rectangle('Position',[21 -180 0 52], 'LineWidth',1);

%% performance
% % ss
rectangle('Position',[1e-1 40 0.05 0], 'LineWidth',1);

% % tracking
rectangle('Position',[1e-1 0 8.9 10], 'LineWidth',1);

% % tracking 2
rectangle('Position',[1e-1 0 0.9 30],'LineWidth',1);

% % tracking 3
rectangle('Position',[1e-1 0 2.4 20],'LineWidth',1);
% 
% % bandwidth
rectangle('Position',[20 -5 0 10], 'LineWidth',1);

% % disturbance rejection
rectangle('Position',[45 -10 100 10], 'LineWidth',1);

% % disturbance rejection
rectangle('Position',[90 -20 100 20], 'LineWidth',1);
%%
% Plot Output Sensitivity
% S = 1 / (1 + KG)
% KS = K / (1 + KG)

% pidsensitivity = k_pid / (1 + k_pid * g_nom);
lagsensitivity = k_lag / (1 + k_lag * g_nom);
f15 = figure(15);
% r = sigmaplot(pidsensitivity);
hold on
r = sigmaplot(lagsensitivity);
grid minor
set(findall(gcf,'type','line'),'linewidth',2);
f15.Position = [100 100 350 300];
setoptions(r, 'FreqUnits', 'Hz');
ylim([-50 270 ])
%%
g = g_nom
% add uncertainty
close all
f7 = figure(7)
s = tf('s');
% sys_bicep = load('bicep_tfs.mat')
hold on
% for m = 1:11
%     % multiplicative uncertainty
%     sysc{m} = d2c(sys_bicep.sys{m});
%     g_err{m} = (sysc{m} - g )/ g ;
%     h = sigmaplot(g_err{m});
%     hold on
% 
% end
for m = 4:11 
    % multiplicative uncertainty
    sysc{m} = d2c(sys{m});
    g_err{m} = (sysc{m} - g )/ g ;
    h = sigmaplot(g_err{m});
    hold on
    
end

% selecting an uncertainty weight
% create weighting function to bound this 
% use form 
min = 1;
max = 0.31;
T = 2;
WI1 = (T*s + min) / ((T/max)* s + 1);         % Sensitivity weight

setoptions(h,'FreqUnits','Hz', 'MagUnits' , 'abs', 'Grid','on');
% xlim([10e-2 300])
h = sigmaplot(WI1, '--k');
set(findall(gcf,'type','line'),'linewidth',1.2);


title({'Multiplicative Uncertainty', 'Voltage Dynamics'})
f7.Position = [100 100 350 300]
ylim([0 1.1])
%%
clear T
f9 = figure(9)
L = g*k_lag;                 % Loop transfer function
S = inv(1+L);                   % Sensitivity 
T = 1-S;                      % Complementary sensitivity 

h = sigmaplot(WI1*T, 'k')
grid on
hold on
yline(0.9, '--', 'Color', [0.6 0.2 0.2],'LineWidth', 1.2)
f9.Position = [100 100 350 300]
title({'Weighted Complementary Sensitivity', 'Voltage Dynamics'})
setoptions(h,'FreqUnits','Hz', 'MagUnits' , 'abs', 'Grid','on');
set(findall(gcf,'type','line'),'linewidth',1.2);


%%

% Create Simulated Closed Loop Step Response
f6 = figure(6)
step(g_nom*k_lag/(1+g_nom*k_lag))
hold on
step(gd*Cd/(1+gd*Cd))
hold on
ts = 1/1000;
Cd = c2d(k_lag, ts)
Cd.Variable = 'z^-1' 
gd = c2d(g_nom, ts);
step(gd*Cd/(1+gd*Cd))

title('Simulated Closed Loop Step Response')
grid minor
set(findall(gcf,'type','line'),'linewidth',2);
f6.Position = [100 100 350 300]

step_info = stepinfo(gd*Cd/(1+gd*Cd))
legend off

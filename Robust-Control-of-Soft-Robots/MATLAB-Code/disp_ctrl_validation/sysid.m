%{
Takes motion capture data and input data from the ROS mcu status 
synchronizes the data - working with 2 channels cap, volt, disp, and charge
signals. 

Plots the data as well as the hysteresis plots associated with the signals.
%}

startup

%% Import data from current path
original_folder_path = pwd;

files = dir('*.csv');
for i=1:length(files)
    data(i) = {readtable(files(i).name, 'Delimiter', ',')};
end

n = 0;
% calibration msg data
cal = data{1+n*9};
% disp data
disp = data{2+n*9};
disp_ref = data{3+n*9};
disp_tri = data{4+n*9};
% error 
error = data{5+n*9};
% kalman filter output
% est_state = data{6};
% mcu ref data
ref = data{6+n*9};
% mcu status data
mcu_stat = data{7+n*9};
% mocap data
theta = data{8+n*9};
theta_ref = data{9+n*9};


%% Align Data
del_t = 0.005; % choose time step 

aligndata;
%%

clr1 = [0.545            0        0.302]
clr2 = [0.286        0.365        0.616]
% tiledlayout(4,1)
% ax1 = nexttile;
close all



f1 = figure(1)
plot(t, charge_aligned(:,1)/4096, 'Color', clr1)
hold on
plot(t, charge_aligned(:,2)/4096, 'Color', clr2)
title('H-Infinity System Response to Desired Orientation')
% ylim([-5 5])
ylabel({'Charge' '(scaled)'})
ylim([-.3 0.3])
xlim([30 50])
% legend('bicep', 'tricep')
grid minor




f2 = figure(2)
plot(t, volt_aligned(:,1)/6000, 'Color', clr1)
hold on
plot(t, volt_aligned(:,2)/6000, 'Color', clr2)
hold on
plot(t, ref_aligned/6000, '--k')
ylabel({'Voltage', '(scaled)'})
xlim([30 50])
ylim([-0.2 1.2])
grid minor



f3 = figure(3);
plot(t, disp_aligned(:,1), 'Color', clr1)
hold on
plot(t, disp_aligned(:,2), 'Color', clr2)
ylabel({'Displacement', '(mm)'})
grid minor
hold on
xlim([30,50])
ylim([-1 8])
hold on
plot(t, err_aligned(:, 3:4), '--k') 


f4 = figure(4)
plot(t, eul_aligned, 'LineWidth', 1.5)
hold on
plot(t, theta_ref_aligned,  '--k')
ylabel({'Theta', '(deg)'})
grid minor
xlim([30 50])
ylim([-1 13])

% ax5 = nexttile; 
% 
% f = figure(5)
% 
% plot(t, eul_aligned - theta_ref_aligned, 'k')


% plot(t, cap_aligned*3.3/4096)
% ylabel('cap (V)')
grid on
xlabel('Time (s)')
% linkaxes([ax1, ax2, ax3, ax4] , 'x')



set(findall(gcf,'type','line'),'linewidth',1.5);

ht = 20*5;
wth = 105*5;
f1.Position = [100 100 wth ht];
f2.Position = [100 100 wth ht];
f3.Position = [100 100 wth ht];
f4.Position = [100 100 wth ht];

% %% plot capacitance
% ht = 300;
% wth = 500;
% a = 20;
% z = 60;
% A = a/del_t;
% Z = z/del_t;
% f8 = figure(8);
% s = scatter(disp_aligned(A:Z,1), cap_aligned(A:Z,1)/4096*3.3, [], 'd', 'filled');
% % s = scatter( cap_filtered(A:Z)/4096*3.3,disp_aligned(A:Z), [], klr(1:end-1,:), 'd', 'filled')
% s.MarkerFaceAlpha = .1;
% hold on
% grid minor
% title('Actuator Displacement vs Capacitance')
% ylabel('self-sensing signal (V)')
% xlabel('displacement (mm)')
% 
% f8.Position = [100 100 wth ht];
% 
% 
% hold on
% folder_path = '/Users/angie/Documents/Research/CurrentProjects/STTR/PhaseII/Code/catkin_ws/src/sttr_phase2/src/MIMO/Kalman/'; % Specify the desired folder path
% cd(folder_path);
% p = load('P.csv');
% p1 = load('P2.csv');
% x = 0:0.1:6;
% % p = polyfit(disp_aligned, cap_aligned/4096*3.3, 6)
% % p = [0 1 1]
% f = polyval(p, x);
% plot(x, f);
% cd(original_folder_path);
% 
% p_inv = polyfit(f, x, 5);
% capx =  1:0.01:3.3;
% cap2disp = polyval(p_inv, capx);
% figure
% 
% 
% % 
% % p_inv = polyfit(cap_aligned(A:Z,1)*3.3/4096, disp_aligned(A:Z,1), 5);
% % f_inv = polyval(p_inv, 1:0.1:3);
% % 
% figure
% plot(cap_aligned(A:Z,1)*3.3/4096, disp_aligned(A:Z,1), 'p')
% 
% hold on
% plot(capx, cap2disp, 'k', 'LineWidth', 2)
% 
% filter_data

%%
close all
f1 = figure(1)
plot(t, charge_aligned(:,1)/4096, 'Color', clr1)
hold on
plot(t, charge_aligned(:,2)/4096, 'Color', clr2)
% ylim([-5 5])
ylabel({'Charge' '(scaled)'})
ylim([-.3 0.3])
xlim([32.3 33.2])
grid minor


set(findall(gcf,'type','line'),'linewidth',2);

f2 = figure(2)
plot(t, volt_aligned(:,1)/6000, 'Color', clr1)
hold on
plot(t, volt_aligned(:,2)/6000, 'Color', clr2)
hold on
plot(t, ref_aligned/6000, '--k')
ylabel({'Voltage', '(scaled)'})
xlim([32.3 33.2])
grid minor


set(findall(gcf,'type','line'),'linewidth',2);

f3 = figure(3);
plot(t, disp_aligned(:,1), 'Color', clr1)
hold on
plot(t, disp_aligned(:,2), 'Color', clr2)
ylabel({'Displacement', '(mm)'})
grid minor
hold on
xlim([32.3 33.2])
ylim([-1 8])
hold on
plot(t, err_aligned(:, 3:4), '--k') 

set(findall(gcf,'type','line'),'linewidth',2);

f4 = figure(4)
plot(t, eul_aligned, 'LineWidth', 1.5)
hold on
plot(t, theta_ref_aligned,  '--k')
ylabel({'Theta', '(deg)'})
xlim([32.3 33.2])
ylim([-1 13])



f1.Position = [100 100 wth/2 ht];
f2.Position = [100 100 wth/2 ht];
f3.Position = [100 100 wth/2 ht];
f4.Position = [100 100 wth/2 ht];

set(findall(gcf,'type','line'),'linewidth',2);

%%
startidx = 32.3/del_t;
stepinfo1 = stepinfo(eul_aligned(6000:6300), del_t*(1:length(eul_aligned(6000:6300))), 10 , 0 , 'SettlingTimeThreshold',0.05)
figure
plot(t(6000:6300), eul_aligned(6000:6300))
    % 
    %      RiseTime: 0.0877
    % TransientTime: 0.8370
    %  SettlingTime: 0.8373
    %   SettlingMin: 9.0897
    %   SettlingMax: 12.4756
    %     Overshoot: 24.7561
    %    Undershoot: 0.8594
    %          Peak: 12.4756
    %      PeakTime: 0.7250



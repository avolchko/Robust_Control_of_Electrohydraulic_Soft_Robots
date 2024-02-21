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


% Iterate through multiple groups of csv files
n = 0;
% calibration msg data
cal = data{1+n*5};
% disp data
disp = data{2+n*5};
disp_tri = data{3+n*5};
% ref data
ref = data{4+n*5};
% mcu status data
mcu_stat = data{5+n*5};
% mocap data
theta = data{6+n*5};


%% Align Data 
% originally was del_t = 0.005
del_t = 0.005; % choose time step 
start = 20;
finish = 670;
aligndata;

%%
clear cap_filtered volt_filtered vel_filtered

% add filtered data on top
order = 3;
dely = 1;
framelen = dely/del_t-1;
cap_filtered = sgolayfilt(cap_aligned,order,5);

order = 3;
dely = 0.05;
framelen = dely/del_t-1;
volt_filtered = sgolayfilt(volt_aligned, order, 5);

order = 3;
dely = .05;
framelen = dely/del_t-1;
vel_filtered = sgolayfilt(vel_aligned, order, 5);

% Plot

bicep_init = [120 180 230 290 335 380 430 480]/del_t;
tricep_init = [560 610 660 700 780 830 940 980]/del_t;
% bicep_fin = bicep_init+10/del_t;
% tricep_fin = tricep_init+10/del_t;
bicep_fin = bicep_init+10/del_t;
tricep_fin = tricep_init+10/del_t;

% for i = 3:6
% 
% f1 = figure(i)
% tiledlayout(3,1)
% ax1 = nexttile;
% plot(t([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)]) - t(bicep_init(i)), disp_aligned([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)],1)/10)
% ylabel({'Displacement', '(scaled)'})
% xlim([0 10])
% grid minor
% 
% 
% 
% hold on
% title('Signals for Modeling Displacement Dynamics')
% 
% 
% ax2 = nexttile;
% plot(t([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)]) - t(bicep_init(i)), disp_aligned([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)],2)/10)
% ylabel({'Displacement', '(scaled)'})
% xlim([0 10])
% grid minor
% % ylim([0.4 0.8])
% 
% ax3 = nexttile;
% plot(t([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)])- t(bicep_init(i)), volt_aligned([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)],1)/6000)
% hold on
% plot(t([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)])- t(bicep_init(i)), ref_aligned([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)],1)/6000, 'k--')
% 
% % legend('bicep', 'tricep', 'bicep ref', 'tricep ref' )
% ylabel({'Voltage', '(scaled)'})
% xlim([0 10])
% grid minor
% 
% % make pretty
% linkaxes([ax1, ax2, ax3] , 'x')
% set(findall(gcf,'type','line'),'linewidth', 1.2);
% px = 700;
% py = 600;
% bx = 520;
% by = 320;
% f1.Position = [px py bx/1.2 by/1.2];
% 
% 
% end
% DMDC and PLOT

clear A B max sys p z input_data state_data segment data_all state_scaled state_data segment data_all alpha_state input_scaled A B D A_scaled B_scaled segment_length
% close all
% number of sample tests
M = 1;
n = 8;

% bicep_init = [120 180 230 290 335 380 430 480]/del_t;
% tricep_init = [560 610 660 700 780 830 940 980]/del_t;
% bicep_fin = bicep_init+10/del_t;
% tricep_fin = tricep_init+10/del_t;


% create state 

% without velocity
% state_pose = [disp_aligned(:, :)/10]; 
% data_all = [volt_aligned(:, :)/6000, state_pose];


% %with velocity
% state_pose = [disp_aligned(1:end-1, :)/10 , vel_filtered(:, :)]; 
% data_all = [volt_aligned(1:end-1, :)/6000, state_pose];

% with velocity and voltage
clear state_pose data_all
state_pose = [disp_aligned(1:end-1, :)/8 , vel_filtered(:, :)/8, volt_filtered(1:end-1, :)/6000]; 
data_all = [volt_filtered(2:end, :)/6000, state_pose];

% clear state_pose data_all
% state_pose = [disp_aligned(1:end-1, :)/8 , volt_aligned(1:end-1, :)/6000]; 
% data_all = [volt_aligned(2:end, :)/6000, state_pose];


for i = 1:n
    time_segment{i} = t([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)]);
	segment{i} = data_all([bicep_init(i):bicep_fin(i), tricep_init(i):tricep_fin(i)], :);
end

% run dmdc on all data 
f1 = figure(1);
for i = 1:size(segment, 2)
    input_data{i} = segment{i}(:,1:2)' ;
    state_data{i} = segment{i}(:,3:8)'; % choose which states we want to incorporate in our model
    [A{i} B{i}] = customdmdc(state_data{i}, input_data{i});
    
    l = length(A{i}(1,:));
    m = length(B{i}(1,:));
    C = [eye(2), zeros(2, length(A{i})-2)];
    D = zeros(2, 2);
    % figure(2)
    % subplot(211)
    % plot(time_segment{i}, state_data{i})
    % hold on
    % subplot(212)
    % plot(time_segment{i}, input_data{i})
    % hold on
    if isnan(A{i}) == 0 
        
        sys{M} = ss(A{i},B{i}, C, D, del_t);
        
        figure(1)
        h = sigmaplot(sys{M});
        
        hold on
        % identify poles and zeros
        [p{i},z{i}] = pzmap(sys{M});
%         figure(2)
        M = M + 1;
%         subplot(n/4, 4, i)
%         step(sys{i}(1,1))
    else
        print('err')
       
   
    end
end


setoptions(h, 'FreqUnits', 'Hz');

f1.Position = [100 100 350 300];
grid minor
%%
figure(1)
s = tf('s');
% sigmaplot(13.5/s, '--k') %% OLD ONE
% actually averaged the 8 tfs
avg_tf = (sys{1}+sys{2}+sys{3}+sys{4}+sys{5}+sys{6}+sys{7}+sys{8})/8;
sigmaplot(avg_tf, '--k') 
% estimate to match that estimate
% g_nom= 2/s;
% sigmaplot(g_nom, '--k') ;
% ylim([-100 0])
xlim([10e-3 5e1])
title({"Displacement Dynamics Sigma Plot:"})
ylabel('Normalized Displacement / Voltage ')
set(findall(gcf,'type','line'),'linewidth',1);

for i = 1:size(sys,2)
    sysc{i} = d2c(sys{i}, 'tustin');
end
sys_selc = sysc{4};
hinf
%% Plot
sys_sel = sys(5);

r = size(A,1);
validation

%% run lqr + i code
% lqri

%% run multi-resolution DMDc code
% mr

%% choose g_nom

% gnom = d2c(avg_tf, 'tustin');

% We are going to use a single linear mapping through the center of the
% displacement data. 
f1 = figure(1);



% We are going to use this C matrix as the g_nom plant
% C = [0.25, 0 0 0 0.036 0 ; 0 0.25 0 0 0 0.036];
% C = [1 0 0 0 0 0 ; 0 1 0 0 0 0];

% k = 0.9: 0.01: 1.1;
delta = 0:0.02:0.2;

% then we are going to use the other data to represent the uncertainty
% in the other plants
for i = 1:8
    % gp(i) = {k(i)*C/(s*eye(6)-sysc{6}.A)*sysc{6}.B};
    % gp(i) = {gnom + delta(i) * eye(2)};
    gp(i) = {sysc{i}};
    h = sigmaplot(gp{i});
    hold on
end

grid minor
xlim([1e-2 1e2]);
% we have to import the sensing data first
% gnom = C/(s*eye(6)-sysc{6}.A)*sysc{6}.B;

sigmaplot(avg_tf, 'k--')
set(findall(gcf,'type','line'),'linewidth',1.2);
f1.Position = [100 100 350 300];
setoptions(h,'FreqUnits','Hz', 'MagUnits' , 'db', 'Grid','on');

% run hinf synthesis on g_nom
hinf

% run mu analysis
uncertain_plants


% %% Create equation for theta given measurements (which idrk)
% th_x = 90;
% array = ones(size(eul_aligned));
% height1 = 393 * array;
% height2 = 387.5 * array;
% pin1 = 2;
% 
% pin2 = 2;
% base1 = (pin1 + 1) * 11 * array;
% base2 = (pin2 + 1) * 11 * array;
% % eul is just the change in angle using the initial measured theta as theta_0
% bicep_angle = th_x-eul_aligned-3;
% len(:,1) = sqrt(height1.^2 + base1.^2 - 2 * height1 .* base1 .* cosd(bicep_angle));
% len(:,2) = sqrt(height2.^2 + base2.^2 - 2 * height2 .* base2 .* cosd(172-bicep_angle));
% 
% f11 = figure(11);
% plot(t, abs(len(:,muscle)-len(1,muscle)));
% hold on
% plot(t, disp_aligned(:,muscle))
% legend('est', 'real')
% grid minor
% title('Measured vs Estimated Bicep Length')
% ylabel('length (mm)')
% xlabel('time (s)')
% ylim([-.1, 12])
% f11.Position = [100 100 ht wth];


%% kalman section
% Q = diag([1e-2 1e-2]); % process noise
% R = diag([1 1]); % sensor noise

% folder_path = '/Users/angie/Documents/Research/CurrentProjects/STTR/PhaseII/Code/catkin_ws/src/sttr_phase2/src/MIMO/Kalman/'; % Specify the desired folder path
% cd(folder_path);
% 
% % Write the cell array to a CSV file
% csvwrite('Q.csv', round(Q,5));
% csvwrite('R.csv', round(R,5));
% 
% csvwrite('Ad.csv', round(A,5));
% csvwrite('Bd.csv', round(B,5));

% csvwrite('P.csv', round(p,5));


% load("mmt_fncn_no_load.mat")

% Change back to the original working directory if necessary
% cd(original_folder_path);

%% Change the current working directory to the desired folder
folder_path = '/Users/angie/Documents/Research/CurrentProjects/STTR/PhaseII/Code/catkin_ws/src/sttr_phase2/src/MIMO/HINF_control_mimo_3/'; % Specify the desired folder path
cd(folder_path);

% Write the cell array to a CSV file
csvwrite('Ad.csv', round(Kd.A,7));
csvwrite('Bd.csv', round(Kd.B,7));
csvwrite('Cd.csv', round(Kd.C,7));
csvwrite('Dd.csv', round(Kd.D,7));

% Change back to the original working directory if necessary
cd(original_folder_path);


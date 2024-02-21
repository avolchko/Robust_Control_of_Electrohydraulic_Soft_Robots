%% use this file to compare to see if the geometry currently in place is accurate
%% compared to the values that we are getting for specified voltages in pretension

% use volt data to determine when system is 'pretensioned'
figure
tiledlayout(3,1)
ax1 = nexttile;
plot(t, disp_aligned)
ax2 = nexttile;
plot(t, eul_aligned)
ax3 = nexttile;
plot(t, volt_aligned)
linkaxes([ax1, ax2, ax3], 'x')

% indices to pull pretensioned data
PT_idx_time  = [118.775 177.015 226.225 289.09 333.925 377.07 425.1 473.33]/del_t;
PT_idx = [23755   35403    45245   57818 66785   75414    85020   94666];


for i = 1:length(PT_idx)
PT_theta(i) = eul_aligned(PT_idx(i));
PT_disp1(i) = disp_aligned(PT_idx(i), 1);
PT_disp2(i) = disp_aligned(PT_idx(i), 2);
end

%% then bring in equation as it is written in ros node

% all measurements are consistent between VS Code and MATLAB
height_1 = 393.1;
height_2 = 386.9;
base_1 = 33;
base_2 = base_1;
act_length = 393;


% sqrt(height_1^2 + base_1^2 - (2 * height_1 * base_1 * cos(pi/2)))
% 375 - sqrt(height_2^2 + base_2^2 - 2 * height_2 * base_2 * cos(pi/2))

init_disp1 = 0;
init_disp2 = 0;

theta_init = acos((act_length^2 - height_1^2 - base_1^2 ) / (-2 * height_1 * base_1)) * 180 /pi;

%
for k = 1:8
theta_ref = PT_theta(k);

theta_rad_1 = (theta_init - theta_ref ) * pi / 180;
ref_length_1 = sqrt(height_1^2 + base_1^2 - 2* height_1 * base_1 * cos(theta_rad_1));

theta_rad_2 = (180 - 2 * ( 90 - theta_init))* pi / 180 - theta_rad_1;
ref_length_2 = sqrt(height_2^2 +base_2^2 - 2 * height_2 * base_2 * cos(theta_rad_2));


% confirmed act_length is same in VS code and in here
ref_disp1 = act_length - ref_length_1;
ref_disp2 = act_length - ref_length_2;

true_disp1 = PT_disp1(k);
true_disp2 = PT_disp2(k);

% calculate difference between the two
err_1(k) = ref_disp1 - true_disp1;
err_2(k) = ref_disp2 - true_disp2;


end
clear max
max(abs(err_1));
max(abs(err_2));
err_1
err_2
% get the output of the ros node and compare that to the actual
% displacements at the given theta

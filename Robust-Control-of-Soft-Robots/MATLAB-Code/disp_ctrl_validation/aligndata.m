
% Create arrays required for timestamping data
mcu_stat_mat = zeros(size(mcu_stat,1), 6);
for i = 1:6
    mcu_stat_mat(:,i) = table2array(mcu_stat(:, i+1));
end
mcu_stat_mat(:,7) = table2array(mcu_stat(:, 11));
mcu_stat_mat(:,8) = table2array(mcu_stat(:, 12));
mcu_stat_mat(:,9) = table2array(mcu_stat(:, 10));

mcu_ref_mat = zeros(size(ref,1), 6);
for i = 1:2
    mcu_ref_mat(:,i) = table2array(ref(:, i+1));
end

eul = table2array(theta(:,2));
dispval = table2array(disp(:,2));
dispval_tri = table2array(disp_tri(:,2));
dispref= table2array(disp_ref(:,2:3));
thetaref= table2array(theta_ref(:,2));
err_arr= table2array(error(:,2:8));
% est= table2array(est_state(:,2:3));

% Convert dd-mm-yy into seconds
lever_arm_time = time2sec(theta.time, ref.time);
mcu_stat_time = time2sec(mcu_stat.time, ref.time);
mcu_ref_time = time2sec(ref.time, ref.time);
disp_time = time2sec(disp.time, ref.time);
disp_time_tri = time2sec(disp_tri.time, ref.time);
disp_time_ref = time2sec(disp_ref.time, ref.time);
theta_time_ref = time2sec(theta_ref.time, ref.time);
err_time = time2sec(error.time, ref.time);
% est_time = time2sec(est_state.time, ref.time);


% Make a time series for each topic
ts_lever = timeseries(eul, lever_arm_time);
ts_mcu = timeseries(mcu_stat_mat, mcu_stat_time);
ts_ref = timeseries(mcu_ref_mat, mcu_ref_time);
ts_disp = timeseries(dispval, disp_time);
ts_disp_tri = timeseries(dispval_tri, disp_time_tri);
ts_disp_ref = timeseries(dispref, disp_time_ref);
ts_theta_ref = timeseries(thetaref, theta_time_ref);
ts_err = timeseries(err_arr, err_time);
% ts_est = timeseries(est, est_time);

% Synchronize data with union of time stamps
TTx = synchronize(ts_lever, ts_mcu, 'union');
TTy = synchronize(ts_mcu, ts_ref, 'union');
TTyy = synchronize(ts_ref, ts_disp, 'union');
TTyyy = synchronize(ts_disp, ts_disp_tri, 'union');
TTyyyy = synchronize(ts_disp, ts_disp_ref, 'union');
TTyyyyy = synchronize(ts_theta_ref, ts_disp, 'union');
TTyyyyyy = synchronize(ts_err, ts_theta_ref, 'union');
% TTyyyyyyy = synchronize(ts_est, ts_err, 'union');
TTz = synchronize(TTx, TTyyyyyy, 'union');

% Create a universal time array
t = TTz.Time(1):del_t:TTz.Time(end); % create time array

% Allocate space for matrices
eul_aligned = zeros(length(t), 1);
mcu_aligned = zeros(length(t), 8);
disp_aligned = zeros(length(t), 2);
disp_ref_aligned = zeros(length(t), 2);
theta_ref_aligned = zeros(length(t), 1);
ref_aligned = zeros(length(t), 2);
err_aligned = zeros(length(t), 7);
% est_aligned = zeros(length(t), 2);

% Interpolate data using time array
eul_aligned(:,1) = interp1(lever_arm_time, eul(:,1), t, 'linear');
ref_aligned(:,:) = interp1(mcu_ref_time, mcu_ref_mat(:,1:2), t, 'linear');
err_aligned(:,:) = interp1(err_time, err_arr, t, 'linear');

disp_aligned(:,1) = interp1(disp_time, dispval, t, 'linear');
disp_aligned(:,2) = interp1(disp_time_tri, dispval_tri, t, 'linear');
disp_ref_aligned(:,1:2) = interp1(disp_time_ref, dispref, t, 'linear');
theta_ref_aligned(:,1) = interp1(theta_time_ref, thetaref, t, 'linear');

% est_aligned(:,1:2) = interp1(est_time, est, t, 'linear');


for i = 1:9
        mcu_aligned(:,i) = interp1(mcu_stat_time, mcu_stat_mat(:,i), t, 'linear');
end

% Create charge and voltage arrays from aligned mcu data
charge_aligned = [mcu_aligned(:,1)-mcu_aligned(:,3), mcu_aligned(:,2)-mcu_aligned(:,4)];
volt_aligned = mcu_aligned(:,[5,6]);
cap_aligned = mcu_aligned(:,[7,8]);
rail_aligned = mcu_aligned(:,9);

% Allocate space for vel calculations
delt = zeros(length(t)-1, 1);
eul_vel = zeros(length(t)-1, 3);

for i = 1:length(t)-1
    delt(i) = t(i+1)-t(i);
    eul_vel(i, :) = (eul_aligned(i+1, :)-eul_aligned(i, :))/delt(i);
end

start_idx = 1
input_signal = [t(start_idx:end)', eul_aligned(start_idx:end, :)];
disp1 = [t(start_idx:end)', disp_aligned(start_idx:end, :)];
ctrl_signal = [t(start_idx:end)', volt_aligned(start_idx:end, :)];



function t2s = time2sec(time, ref)
    time_ref = datetime(time, 'InputFormat', 'yyyy/MM/dd/HH:mm:ss.SSSSSS');
    time_ref_x = posixtime(time_ref);
    
    ref_ref = datetime(ref, 'InputFormat', 'yyyy/MM/dd/HH:mm:ss.SSSSSS');
    ref_ref_x = posixtime(ref_ref);
    
    t2s = time_ref_x - ref_ref_x(1);
end


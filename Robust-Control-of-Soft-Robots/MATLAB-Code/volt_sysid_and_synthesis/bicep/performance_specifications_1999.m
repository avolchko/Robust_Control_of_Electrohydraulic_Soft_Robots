

% calculate percentage ss error
ss_magnitude = 40; % db
ss_abs_magnitude = db2mag(ss_magnitude);
ss_error = 1/ss_abs_magnitude;
SSE = 100 *ss_error % percent ss error


% calculate tracking error percentages

tracking1_magnitude = 24; % db
tracking1_abs_magnitude = db2mag(tracking1_magnitude);
tracking1_error = 1/tracking1_abs_magnitude;
T1E = 100 * tracking1_error % percent ss error

tracking2_magnitude = 17; % db
tracking2_abs_magnitude = db2mag(tracking2_magnitude);
tracking2_error = 1/tracking2_abs_magnitude;
T2E = 100 * tracking2_error % percent ss error

tracking3_magnitude = 10; % db
tracking3_abs_magnitude = db2mag(tracking3_magnitude);
tracking3_error = 1/tracking3_abs_magnitude;
T3E = 100 * tracking3_error % percent ss error


% calculate bandwidth
BW = 100;


% calculate disturbance rejection
distrej1_magnitude = -5; % db
distrej1_error = db2mag(distrej1_magnitude);
DR1 = 1/ distrej1_error % percent ss error

distrej2_magnitude = -10; % db
distrej2_error = db2mag(distrej2_magnitude);
DR2 = 1/ distrej2_error % percent ss error

% calculate percent overshoot

% define phase margin
PM = 72;
ksi = PM / 100;

Mp = exp((-pi*ksi)/sqrt(1-ksi^2));

OS = 100* Mp

% check ksi is = check
check = sqrt(log(Mp)^2/(pi^2+log(Mp)^2));


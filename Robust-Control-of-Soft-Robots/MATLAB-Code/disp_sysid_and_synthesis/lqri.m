% create lqr controller (follow murray's notes)

% add a new state to the system that is given by xidot = v - vd
% construct ctrl law by computing lqr gains for the augmented system

% augment plant matrices so that we have a new state of error: zdot = r - y

Ai = [sys_sel.A , zeros(size(sys_sel.A,1), size(sys_sel.C,1));
      sys_sel.C , zeros(size(sys_sel.C, 1),(size(sys_sel.C, 1)))];

Bi = [sys_sel.B ; zeros(size(sys_sel.C, 1), size(sys_sel.B,2))];

Ci = [sys_sel.C , zeros(size(sys_sel.C, 1),(size(sys_sel.C, 1)))];

Di = sys_sel.D;

Qi = diag([10, 10, 1, 1, 200, 200]);
% penalize xi by increasing qi

Ri = diag([10, 50]);
% penalize ui by increasing ri

K = lqr(Ai, Bi, Qi, Ri);

Kp = K(:,1:4)
Ki = K(:, 5:6)
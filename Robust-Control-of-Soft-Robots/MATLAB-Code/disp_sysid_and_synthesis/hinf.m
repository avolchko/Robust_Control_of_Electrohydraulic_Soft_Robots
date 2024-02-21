%% Controller synthesis
clear S T L K CL
% W1 is a NxN where N is the number of inputs to the controller
sys_selc = sysc{4};
% sys_selc = d2c(sys_sel, 'tustin');

s = tf('s');
w0 = 0.8*2*pi;                         % Desired closed-loop bandwidth (rad/s)
Ai = 1/65;                        % Desired disturbance attenuation inside bandwidth (1/A X)
M = 2;                            % Desired bound on hinfnorm(S) & hinfnorm(T)
W1a = (s/M^0.5+w0)^2/(s+w0*Ai^0.5)^2;         % Sensitivity weight
% W1a = (s/M+w0)/(s+w0*Ai); 


w0 = 0.6*2*pi;                         % Desired closed-loop bandwidth (rad/s)
Ai = 1/10;                        % Desired disturbance attenuation inside bandwidth (1/A X)
M = 2;                            % Desired bound on hinfnorm(S) & hinfnorm(T)
% W1b = (s/M+w0)/(s+w0*Ai);         % Sensitivity weight
W1b = (s/M^0.5+w0)^2/(s+w0*Ai^0.5)^2; 

W1 = blkdiag([W1a, 0; 0 W1b]); 

% W2 is an MxM where M is the number of outputs from the controller

W2a =  (4*(s+3)/(s+12))^2;     % K*S weight % can I adjust this however I see fit?


W2b = (1.9*(s+5)/(s+12))^2;      % K*S weight % can I adjust this however I see fit?

W2 = blkdiag([W2a, 0; 0 W2b]); 


% W3 is a NxN where N is the number of inputs to the controller
W3a = 1.1*(2+s)/((s+3));            % Empty control weight (weight on T)
% W3a = 1/1.3622 + s/(2*pi*0.7);

W3b = 0;                         % Empty control weight (weight on T)

W3 = blkdiag([W3a, 0; 0 W3a]);

% plots min(Ny, Nu)             # of lines

clear K
GAMTRY = 2;
% opts = hinfsynOptions('AutoScale','on')
[K,CL,GAM,INFO] = mixsyn(sys_selc,W1,W2,W3);
GAM % we want gamma to be close to or < 1 

f27 = figure(27);
tiledlayout(4,1)
L = sys_selc*K;                 % Loop transfer function
I = eye(size(L));
S = 1/(I+L);                   % Sensitivity 
S = feedback(I,L); 
T = I-S;                      % Complementary sensitivity 

% using / works better than 
f1 = figure(1)
s = sigmaplot( inv(W1a),'r--', inv(W1b),'b--', S(1,1), 'r-', S(2,2), 'b-', {1e-1,1e2}); grid on;
xlim([2e-2 10])

setoptions(s, 'MagUnits', 'dB', 'FreqUnits', 'Hz')
KS = K*S;

f2 = figure(2)
s = sigmaplot(inv(W2a),'r--', inv(W2b), '--b', KS(1, 1) ,'r-', KS(2,2) ,'b-',{1e-1,1e2}); grid on;
xlim([2e-2 10])
setoptions(s,  'FreqUnits', 'Hz')


f3 = figure(3)

s = sigmaplot(inv(W3a), '--r', T(1,1),'-r', T(2,2), '-b'); grid on; hold on;
% sigma(inv(W3b),'r--', T(2,:), '-r'); grid on; 
xlim([2e-2 10])

setoptions(s,  'FreqUnits', 'Hz')

% legend('1/{W_3}', 'T');
set(findall(gcf,'type','line'),'linewidth',1.5);

wth = 500
ht = 150
f1.Position = [100 100 wth ht];
f2.Position = [100 100 wth ht];
f3.Position = [100 100 wth ht];

%%
% Open loop singular values
f28 = figure(28); clf;
s = sigmaplot(sys_selc,'b', K, sys_selc*K ,{1e-1,1e2}); grid on; 
% legend('\sigma(G)','\sigma(K)','\sigma(G*K)'); title('Open Loop Singular Values');
setoptions(s,  'FreqUnits', 'Hz')

% useful to have the following to see the poles of the closed loop tf
[z_CL, p_CL] = ss2zp(T.A, T.B, T.C, T.D , 1);


set(findall(gcf,'type','line'),'linewidth',1.5);

ht = 550;
wth = 350;
f28.Position = [100 100 wth ht/2];

% figure
% step(T)

% convert cont K into discrete K at 200 Hz
ts = 1/200;
Kd = c2d(K, ts);

GAM

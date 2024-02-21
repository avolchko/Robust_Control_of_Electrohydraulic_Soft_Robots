
if n == 10
    % all data
%     a = 270;
%     z = 750;
a = a
z = z
    clr1 = [0.80000         0    0.7];

end

if n == 1
    % 2kV
    a = 65;
    z = 100;
    clr1 = [0.80000         0    0.7];

end

if n == 2
    % 2.5 kV
a = 375;
z = 395; 
    clr1 = [1.1000    0.4120    0.7060];
end

if n == 3
    % 3 kV
    a = 430;
    z = 450;
    clr1 = [0.1920    0.7    0.4];

end

if n == 4
    % 3.5 kV
    a = 495;
    z = 515;

    clr1 = [0    0.5350    1.0000];

end


if n == 5
    % 4 kV
    a = 555;
    z = 575;

    clr1 = [0.4980    0.3140    0.7610];

end

% if n == 6
%     % 4.5 kV
%     a = 590;
%     z = 610;
% 
%     clr1 = [0.9310    0.1020    0.1020];
% 
% end

if n == 7
    % 4.5 kV
    a = 655;
    z = 680;


    clr1 =  [0.9570    0.4550    0.1690];

end

if n == 8
    % 5 kV
    a = 715;
    z = 735;

    clr1 = [0.1920    0.5840    0.5570];

end



clr2 = clr1;

f1 = figure(n);

tiledlayout(4,1)
ax1 = nexttile;
plot(t, disp_aligned(:,:))
hold on
% plot(t, -disp_aligned(:,2)) % make sure this is being read right from mocap to ros
colororder({'k', 'b'})
xlim([a z])
ylabel('displacement (mm)')
% legend('biceps signal', 'triceps signal', 'Orientation', 'horizontal')
xlabel('time (s)')
grid minor
hold on

ax2 = nexttile;
plot(t, cap_aligned(:,:)/4096*3.3, 'c')
hold on

plot(t, cap_filtered(:,:)/4096*3.3, '--k')
colororder({'k', 'b'})
hold on

xlim([a z])
ylabel('capacitance reading (V)')
% legend('biceps signal', 'triceps signal', 'Orientation', 'horizontal')
xlabel('time (s)')
grid minor

ax3 = nexttile;
plot(t, volt_aligned(:,:)/1000)
colororder({'k', 'b'})
hold on
plot(t, rail_aligned(:,:)/1000)
% plot(t, ref_aligned,'--k')
xlim([a z])
ylabel('voltage reading (kV)')

plot(t, volt_filtered(:,:)/1000, '--k')
hold on

% legend('actual voltage', 'ref voltage')
% legend('biceps signal', 'triceps signal', 'Orientation', 'horizontal')
xlabel('time (s)')
grid minor

ax4 = nexttile;
plot(t, charge_aligned(:,:)/4096*100)
colororder({'k', 'b'})
hold on
xlim([a z])
ylabel('charge reading (%)')
% legend('biceps signal', 'triceps signal', 'Orientation', 'horizontal')
xlabel('time (s)')
ylim([-100 100])
grid minor
hold on


set(findall(gcf,'type','line'),'linewidth',2.2);

f1.Position = [100 100 550 610];

linkaxes([ax1, ax2, ax3, ax4] , 'x')
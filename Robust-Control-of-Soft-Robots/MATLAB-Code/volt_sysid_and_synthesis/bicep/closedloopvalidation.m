CL = sim("closedloop_voltage_simulation.slx");
%

clr1 = [0.5450         0    0.3020];

fsim = figure(1);
tiledlayout(2,1);
ax1 = nexttile;
plot(CL.voltage, 'k','LineWidth',2);

hold on
plot(CL.reference, '--', 'Color', clr1, 'LineWidth',2);
ylabel({'Voltage', '(scaled)'})
grid minor
title('Simulated Closed Loop Response')

ax2 = nexttile;
plot(CL.charge, 'k', 'LineWidth',2);

ylabel({'Charge', '(scaled)'})
grid minor
title ''
ylim([-0.3 0.3])
set(findall(gcf,'type','line'));

xpos = 100;
ypos = 100;
wdth = 350;
hght = 300;
fsim.Position = [xpos, ypos, wdth, hght];


linkaxes([ax1, ax2] , 'x')



%%
fsim2 = figure(2);
tiledlayout(2,1);

ax1 = nexttile;
plot(CL.voltage2*6, 'k', 'LineWidth',1)

hold on
plot(CL.reference2*6, '--', 'LineWidth',1)
ylabel('voltage (kV)')
grid minor
title('Closed Loop Response to a Step Input')

ax2 = nexttile;
plot(CL.charge2, 'k', 'LineWidth',1)

ylabel({'charge', '(normalized)'})
grid minor
title ''
ylim([-60 60])

xpos = 100;
ypos = 100;
wdth = 300;
hght = 250;
fsim2.Position = [xpos, ypos, wdth, hght];

set(findall(gcf,'type','line'),'linewidth',2);
linkaxes([ax1, ax2] , 'x')
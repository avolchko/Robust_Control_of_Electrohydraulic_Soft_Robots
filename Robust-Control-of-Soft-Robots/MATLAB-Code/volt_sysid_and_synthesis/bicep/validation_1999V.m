% use this code to validate the model with the real world measurements
% using the true input to the system

gnomss = ss(g_nom);
% sys_sel = c2d(gnomss, del_t);
h = figure(1);


for i = 1
x0 =  state_data{i}(1, :);
u = input_data{i};
tsim = t(start_idx(i)):dt:t(start_idx(i))+dt*(length(u)-1); % in seconds

sys_sel = sys{i};

% plot to compare
f1 = figure(1);
ylabel({'Voltage',  '(scaled)'})
% plot(tsim - t(start_idx(i)), state_data{i}, 'r')
hold on
end
grid minor
for i = 4:8
sys_sel = sys{i};
figure(1)

[ly, lt, lx] = lsim(sys_sel, u, tsim', x0);
yl = double(ly);
plot(tsim-t(start_idx(1)), yl, '-k')
hold on
ylim([-0.4 0.4])

set(findall(gcf,'type','line'),'linewidth',1.2);

f2 = figure(2);
plot(tsim - t(start_idx(1)), input_data{i})
ylabel({'Charge',  '(scaled)'})
ylim([-0.1 0.1])

end
grid minor

px = 700;
py = 600;
bx = 600;
by = 200;
f1.Position = [px py bx by];

f2.Position = [px py bx by/2];



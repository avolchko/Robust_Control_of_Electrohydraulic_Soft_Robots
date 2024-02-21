%{
Takes motion capture data and input data from the ROS mcu status 
synchronizes the data
returns a few transfer functions depending on my choice of time interval
divisions & develops an h infinity dynamic controller to be used on the system 
%}

%% import data from sysid folder
startup;


files = dir('*.csv');
for i=1:length(files)
    data_arr(i) = {readtable(files(i).name, 'Delimiter', ',')};
end

% identify data files (in alphabetical order)
% ref data
data = data_arr{1};


% step 0 to 1000
start = 0
finish = 1000


% convert from microseconds to seconds
conversion = 0.000001;

init = start/conversion;
fin = finish/conversion;

t = (data.x_timestamp-data.x_timestamp(1))*conversion - start;

% specify time between data, dt
dt = 0.0005;

%% Plot all of the data used in DMDc

% find all values when reference is set to zero
ref_idx_arr = find(data.x_voltref == 0);

% initialize index
k = 2;

% initialize start index array
final_idx = ones(11,1);
final_idx(1) = ref_idx_arr(1,1) - 1;

% find all indices
for i = 1:length(ref_idx_arr) - 1
    %find big jumps in data 
    if (ref_idx_arr(i+1) - ref_idx_arr(i)) > 100
        final_idx(k) = ref_idx_arr(i+1) - 1;
        k = k+1;
    end
end

% determine final value of each segment
% start_idx = final_idx - 9.8/dt;
start_idx = final_idx - 9.8/dt;


% plot this data
f1 = figure(1);
clr1 = [0.5450         0    0.3020];



tile = tiledlayout(1,11,'TileSpacing','compact');
bgAx = axes(tile,'XTick',[],'YTick',[],'Box','off');
bgAx.Layout.TileSpan = [1 11];
ax1 = axes(tile);

plot(t(start_idx(1):final_idx(1)),  data.x_volt(start_idx(1):final_idx(1))/6000, '-', 'Color', clr1)
hold on
plot(t(start_idx(1):final_idx(1)),  data.x_voltref(start_idx(1):final_idx(1))/6000, '--k')
ax1.Box = 'off';
ax1.XAxis.Visible = 'off';
xlim(ax1,[t(start_idx(1)) t(final_idx(1))])
ylim(ax1, [0 1000]/6000)


for k = 2:11
    ax2 = axes(tile);
    ax2.Layout.Tile = k;

    plot(t(start_idx(k):final_idx(k)),  data.x_volt(start_idx(k):final_idx(k))/6000, '-', 'Color', clr1)
    hold on
    plot(t(start_idx(k):final_idx(k)),  data.x_voltref(start_idx(k):final_idx(k))/6000, '--k')
    ax2.YAxis.Visible = 'off';
    ax2.XAxis.Visible = 'off';
    ax2.Box = 'off';
    xlim(ax2,[t(start_idx(k)) t(final_idx(k))])
    ylim(ax2, ([500*k-500 500*k+500])/6000)

end
    
    set(findall(gcf,'type','line'),'linewidth',1.2);
    f1.Position = [100 300 2000 100];


    % plot this data
f2 = figure(2);

tile = tiledlayout(1,11,'TileSpacing','compact');
bgAx = axes(tile,'XTick',[],'YTick',[],'Box','off');
bgAx.Layout.TileSpan = [1 11];
ax1 = axes(tile);
plot(t(start_idx(1):final_idx(1)), (data.x_charge(start_idx(1):final_idx(1)) - data.x_discharge(start_idx(1):final_idx(1)))/4096, '-k')
ax1.Box = 'off';
ax1.XAxis.Visible = 'off';
xlim(ax1,[t(start_idx(1)) t(final_idx(1))])
ylim(ax1, [-0.1 0.1])


for k = 2:11
    ax2 = axes(tile);
    ax2.Layout.Tile = k;
    plot(t(start_idx(k):final_idx(k)),  (data.x_charge(start_idx(k):final_idx(k)) - data.x_discharge(start_idx(k):final_idx(k)))/4096, '-k')
    ax2.YAxis.Visible = 'off';
    ax2.XAxis.Visible = 'off';
    ax2.Box = 'off';
    xlim(ax2,[t(start_idx(k)) t(final_idx(k))])
    ylim(ax2, [-0.1 0.1])

end
    
    set(findall(gcf,'type','line'),'linewidth',1.2);
    f2.Position = [100 100 2000 100];

    %% create individidual figures and export as svgs
    % close all
    % for k = 1:11
    %    f1 = figure;
    %    plot(t(start_idx(k):final_idx(k)),  (data.x_charge(start_idx(k):final_idx(k)) - data.x_discharge(start_idx(k):final_idx(k)))/4096, '-k')
    %    f1.Position = [100 100 200 100];
    %    xlim([t(start_idx(k)) t(final_idx(k))])
    %    ylim([-0.1 0.1])
    %    set(findall(gcf,'type','line'),'linewidth',1.2);
    %    % Define the filename for the SVG file
    %    filename = sprintf('charge%d.svg', k);
    % 
    %    % Export the figure as an SVG file
    %    saveas(gcf, filename, 'svg');
    % end
    % 
    % close all
    % for k = 1:11
    %    f1 = figure;
    %     plot(t(start_idx(k):final_idx(k)),  data.x_volt(start_idx(k):final_idx(k))/6000, '-', 'Color', clr1)
    %     hold on
    %     plot(t(start_idx(k):final_idx(k)),  data.x_voltref(start_idx(k):final_idx(k))/6000, '--k')
    %     f1.Position = [100 100 200 150];
    %     xlim([t(start_idx(k)) t(final_idx(k))])
    %     ylim([500*k-500 500*k+500]/6000)
    %     set(findall(gcf,'type','line'),'linewidth',1.2);
    %    % Define the filename for the SVG file
    %    filename = sprintf('volt%d.svg', k);
    % 
    %    % Export the figure as an SVG file
    %    saveas(gcf, filename, 'svg');
    % end

    %% Run sysid for all of the segments of data

    M=1; 

    % run dmdc on all data  and plot corresponding tf
    f3 = figure(3);
    for i = 1:3
    input_data{i} = (data.x_charge(start_idx(i):final_idx(i)) - data.x_discharge(start_idx(i):final_idx(i)))/4096;
    state_data{i} = data.x_volt(start_idx(i):final_idx(i))/6000;
    [A{i} B{i}] = customdmdc(state_data{i}', input_data{i}');
    l = length(A{i}(1,:));
    m = length(B{i}(1,:));
    k_lag = eye(l);
    D = zeros(l, m);
    
    if isnan(A{i}) == 0
        sys{M} = ss(A{i},B{i}, k_lag, D, dt);
        h = sigmaplot(sys{M});
        setoptions(h, 'FreqUnits', 'Hz');
        hold on
        M = M + 1;
    else
        print('err')
    end

    end
    xlim([1e-2 1e3])
    legend '500' '1000' '1500' '2000' '2500' '3000' '3500' '4000' '4500' '5000' '5500' 'Gnom'
        legend('Location','eastoutside')
    set(findall(gcf,'type','line'),'linewidth',1.2);
    
    grid minor
    s = tf('s');
    % avg_tf = (sys{1}+sys{2}+sys{3}+sys{4}+sys{5}+sys{6}+sys{7})/7;
    avg_tf = (sys{1}+sys{2}+sys{3})/3;
    sigmaplot(avg_tf, 'm') 
    % estimate to match that estimate
    % g_nom= 35/s;
    g_nom = 350/s;
    sigmaplot(g_nom, '--k') ;
    ylim([-30 10])
    xlim([2e0 1.5e2])
    title({"Voltage Dynamics:", "Triceps Actuator"})
    ylabel('Normalized Voltage / Charge')
    set(findall(gcf,'type','line'),'linewidth',1.2);


    f3.Position = [100 100 350 300];

    % save the tf data so that I can compile the two in one file
    save('tricep_tfs', 'sys')
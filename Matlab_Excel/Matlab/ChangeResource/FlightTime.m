x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [21.0827	16.1608	13.8919	12.6167	11.6798	10.9173	10.4683	10.2127	9.82867	9.34568	9.204]; % global tsp
% y2 = [21.3377	17.1505	15.0972	13.998	13.1425	12.485	12.03	11.8292	11.5195	11.0296	10.7994]; % local tsp
y3 = [21.3377	17.1505	15.0972	13.998	13.1425	12.485	12.03	11.8292	11.5195	11.0296	10.7994]; % iterative
y4 = [19.2932	15.7497	13.6034	12.467	11.6035	10.988	10.518	10.1593	9.89127	9.62445	9.43912]; % inter-cell
y5 = [22.0667	18.1313	16.5647	15.4415	14.5097	13.8764	13.4021	12.9581	12.6822	12.0602	12.0618];% GELSGA
y6 = [19.0036	15.4467	13.2557	12.0697	11.1542	10.4925	9.97723	9.58883	9.27468	8.9823	8.76441];% CVRP
close all;
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
plot(x,y4,'-kv', 'LineWidth',2);
hold on;
grid on;
% plot(x,y2,'-bs', 'LineWidth',2);
plot(x,y3,'-rd', 'LineWidth',2);
plot(x,y1,'-mo', 'LineWidth',2);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
plot(x,y6,'->','Color',[1 0.5 0], 'LineWidth',2);
xlabel('Amount of resource per flight');
ylabel('Flight time (h)');
legend('Inter-cell','Iterative strategy','Global TSP','GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
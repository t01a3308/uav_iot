x = [40 50 60 70 80];
y1 = [3.18359	4.05859	4.84131	5.36621	6.21851]; % global tsp
y2 = [3.40015	4.38428	5.22632	5.99121	7.12061]; % local tsp
y3 = [3.6709	4.53101	5.49658	6.29419	6.98291]; % iterative
y4 = [3.31445	4.01367	4.80957	5.47852	6.04492]; % inter-cell
y5 = [3.26025	4.13086	5.09546	5.74463	6.50635];% GELSGA
close all;
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
plot(x,y1,'-kv', 'LineWidth',2);
hold on;
grid on;
plot(x,y2,'-bs', 'LineWidth',2);
plot(x,y3,'-rd', 'LineWidth',2);
plot(x,y4,'-mo', 'LineWidth',2);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
xlabel('Total site');
ylabel('Total data (MB)');
legend('Global TSP','Local TSP','Iterative strategy','Inter-cell','GELSGA');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([40 50 60 70 80]);
xlim([40 80]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
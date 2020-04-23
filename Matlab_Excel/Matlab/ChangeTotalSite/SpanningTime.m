x = [40 50 60 70 80];
y1 = [23.9037	32.6707	37.0057	35.2638	46.5991]; % global tsp
y2 = [26.3205	32.6161	38.4725	43.5438	52.3585]; % local tsp
y3 = [27.017	30.9688	37.0091	40.6668	45.4687]; % iterative
y4 = [24.3806	27.0416	31.7607	35.1722	35.6611]; % inter-cell
y5 = [24.1688	31.4761	38.0123	41.9057	49.1136];% GELSGA
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
ylabel('Spanning time (s)');
legend('Global TSP','Local TSP','Iterative strategy','Inter-cell','GELSGA');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([40 50 60 70 80]);
xlim([40 80]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
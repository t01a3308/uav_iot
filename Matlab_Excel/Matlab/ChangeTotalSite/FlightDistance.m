x = [40 50 60 70 80];
y1 = [25.4685	31.9809	37.8462	41.3231	45.5341]; % global tsp
y2 = [30.0745	41.0657	49.4748	54.8183	69.1083]; % local tsp
y3 = [28.4743	36.7877	44.3225	49.4074	53.1951]; % iterative
y4 = [28.259	33.9061	41.0595	44.1818	48.3202]; % inter-cell
y5 = [27.0495	34.508	46.3078	49.0915	52.2389];% GELSGA
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
ylabel('Flight distance (km) ');
legend('Global TSP','Local TSP','Iterative strategy','Inter-cell','GELSGA');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([40 50 60 70 80]);
xlim([40 80]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
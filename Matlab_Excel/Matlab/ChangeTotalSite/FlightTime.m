x = [40 50 60 70 80];
y1 = [3.15718	3.95155	4.68458	5.33076	5.96315]; % global tsp
y2 = [3.41307	4.45627	5.33061	6.08049	7.27282]; % local tsp
y3 = [3.32417	4.2186	5.04437	5.77989	6.38875]; % iterative
y4 = [3.3122	4.05851	4.86309	5.48958	6.11793]; % inter-cell
y5 = [3.24501	4.09195	5.15466	5.76234	6.33564];% GELSGA
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
ylabel('Flight time (h)');
legend('Global TSP','Local TSP','Iterative strategy','Inter-cell','GELSGA');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([40 50 60 70 80]);
xlim([40 80]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
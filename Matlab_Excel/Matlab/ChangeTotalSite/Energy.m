x = [40 50 60 70 80];
y1 = [4.2785	5.35272	6.36529	7.26107	8.1186]; % global tsp
y2 = [4.60093	5.9503	7.17933	8.19474	9.77035]; % local tsp
y3 = [4.44594	5.65084	6.76222	7.77056	8.60949]; % iterative
y4 = [4.3659	5.44879	6.5792	7.43761	8.29817]; % inter-cell
y5 = [4.39064	5.48726	6.91997	7.76308	8.54694];% GELSGA
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
ylabel('Energy consumption (MJ) ');
legend('Global TSP','Local TSP','Iterative strategy','Inter-cell','GELSGA');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([40 50 60 70 80]);
xlim([40 80]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
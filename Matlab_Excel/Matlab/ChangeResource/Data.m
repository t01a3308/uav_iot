x = [250 300 350 400 450 500 550 600];
y3 = [26.2244	25.086	24.2907	23.6548	23.4631	23.0276	22.6797	22.3665]; % iterative
y4 = [23.3615	21.9047	20.2957	19.1925	18.537	17.9787	17.5029	17.1736]; % inter-cell
y5 = [23.1585	21.8971	21.1032	20.4908	20.2418	19.9587	19.4453	19.3552];% GELSGA
y6 = [21.9451	20.9766	20.1322	19.4913	19.0646	18.7109	18.4873	18.0939];% CVRP
close all;
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
plot(x,y4,'-kv', 'LineWidth',2);
hold on;
grid on;
plot(x,y3,'-rd', 'LineWidth',2);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
plot(x,y6,'->','Color',[1 0.5 0], 'LineWidth',2);
xlabel('Amount of resource per flight');
ylabel('Total data (MB)');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([250 300 350 400 450 500 550 600]);
xlim([250 600]);
hold off;
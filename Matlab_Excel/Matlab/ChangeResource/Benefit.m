x = [250 300 350 400 450 500 550 600];
y3 = [3.0383	3.45329	3.71671	3.97256	4.07706	4.20367	4.37279	4.46291]; % iterative
y4 = [3.36447	3.74234	4.05015	4.26982	4.43783	4.62489	4.77201	4.88302]; % inter-cell
y5 = [2.87551	3.28655	3.59465	3.82043	3.96918	4.12161	4.22857	4.27935];% GELSGA
y6 = [3.16081	3.47843	3.76755	3.95582	4.09559	4.21553	4.31758	4.3778];% CVRP
y3 = y3*1000000;
y4 = y4*1000000;
y5 = y5*1000000;
y6 = y6*1000000;
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
ylabel('Benefit');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([250 300 350 400 450 500 550 600]);
xlim([250 600]);
hold off;
x = [300 350 400 450 500 550 600 650 700];
y3 = [27.7101	31.1482	33.0933	40.7779	45.7457	51.0859	54.6227	58.5753	63.1537]; % iterative
y4 = [23.1127	24.4258	25.8219	28.215	32.3217	35.9361	39.3561	45.5882	51.0541]; % inter-cell
y5 = [31.8397	37.4692	38.507	43.8794	50.3875	57.723	60.022	66.1958	69.3224];% GELSGA
y6 = [27.6507	32.6847	35.1409	41.2971	47.678	52.5304	55.9822	62.0056	66.833]; %CVRP
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
xlabel('Total sites');
ylabel('Spanning time (s)');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 350 400 450 500 550 600 650 700]);
xlim([300 700]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
x = [300 350 400 450 500 550 600 650 700];
y3 = [2.65844	3.07976	3.52802	3.923	4.20367	4.51587	4.85638	5.22223	5.45753]; % iterative
y4 = [3.0057	3.4492	3.88436	4.31201	4.62489	4.91701	5.23414	5.59157	5.89191]; % inter-cell
y5 = [2.6641	2.92346	3.36316	3.84426	4.12161	4.29361	4.52322	4.65584	4.75085];% GELSGA
y6 = [2.80176	3.1686	3.54045	3.9153	4.21553	4.4794	4.77318	5.03775	5.25597]; %CVRP
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
xlabel('Total sites');
ylabel('Benefit');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 350 400 450 500 550 600 650 700]);
xlim([300 700]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
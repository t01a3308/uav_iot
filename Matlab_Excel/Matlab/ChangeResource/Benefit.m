x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [4.44237	5.21452	5.60427	5.8129	5.99524	6.17831	6.26098	6.28419	6.39179	6.51531	6.51929]; % global tsp
% y2 = [4.65333	5.30006	5.74551	5.95973	6.12829	6.27383	6.35682	6.42901	6.49778	6.56588	6.58116]; % local tsp
y3 = [0.893661	2.6275	3.49016	3.97377	4.31494	4.59015	4.79692	4.90404	5.03364	5.19323	5.30929]; % iterative
y4 = [1.57456	3.04779	3.94556	4.38302	4.73985	5.0027	5.18012	5.34547	5.4497	5.54781	5.61846]; % inter-cell
y5 = [4.13553	4.86358	5.18553	5.38892	5.55316	5.67235	5.75741	5.80945	5.87143	5.96696	5.97828];% GELSGA
y6 = [1.43446	2.89633	3.80343	4.23454	4.60352	4.86942	5.06343	5.2242	5.33671	5.4463	5.50924];% CVRP
y1 = y1*1000000;
% y2 = y2*1000000;
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
% plot(x,y2,'-bs', 'LineWidth',2);
plot(x,y3,'-rd', 'LineWidth',2);
% plot(x,y1,'-mo', 'LineWidth',2);
% plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
plot(x,y6,'->','Color',[1 0.5 0], 'LineWidth',2);
xlabel('Amount of resource per flight');
ylabel('Total benefit');
legend('Inter-cell','Iterative strategy','CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
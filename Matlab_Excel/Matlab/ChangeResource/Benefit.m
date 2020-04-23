x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [4.44237	5.21452	5.60427	5.8129	5.99524	6.17831	6.26098	6.28419	6.39179	6.51531	6.51929]; % global tsp
% y2 = [4.65333	5.30006	5.74551	5.95973	6.12829	6.27383	6.35682	6.42901	6.49778	6.56588	6.58116]; % local tsp
y3 = [4.70473	5.48244	5.90981	6.14307	6.31405	6.44596	6.5374	6.60814	6.67753	6.73377	6.76876]; % iterative
y4 = [4.88237	5.52668	5.98711	6.21684	6.39041	6.53898	6.62296	6.70289	6.77239	6.8401	6.85394]; % inter-cell
y5 = [4.13553	4.86358	5.18553	5.38892	5.55316	5.67235	5.75741	5.80945	5.87143	5.96696	5.97828];% GELSGA
y6 = [4.65333	5.30006	5.74551	5.95973	6.12829	6.27383	6.35682	6.42901	6.49778	6.56588	6.58116];% CVRP
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
plot(x,y1,'-mo', 'LineWidth',2);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
plot(x,y6,'->','Color',[1 0.5 0], 'LineWidth',2);
xlabel('Amount of resource per flight');
ylabel('Total benefit');
legend('Inter-cell','Iterative strategy','Global TSP','GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
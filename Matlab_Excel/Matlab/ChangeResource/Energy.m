x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [27.0956	20.8903	18.0305	16.4227	15.2418	14.2798	13.7174	13.3919	12.9081	12.3024	12.1195]; % global tsp
% y2 = [27.4452	22.1692	19.5819	18.1969	17.1188	16.2904	15.7171	15.4641	15.0738	14.4565	14.1665]; % local tsp
y3 = [27.4437	22.1684	19.5833	18.1956	17.1211	16.2929	15.7211	15.4664	15.0742	14.4569	14.1679]; % iterative
y4 = [24.8458	20.3808	17.6773	16.25	15.1593	14.3855	13.7932	13.3396	13.0067	12.6717	12.4388]; % inter-cell
y5 = [28.327	23.3728	21.3962	19.9822	18.8091	18.0126	17.4188	16.8569	16.5161	15.7322	15.7303];% GELSGA
y6 = [24.4737	19.9905	17.232	15.738	14.5866	13.7518	13.106	12.6156	12.2181	11.8551	11.5764];% CVRP
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
ylabel('Energy consumption (MJ) ');
legend('Inter-cell','Iterative strategy','Global TSP','GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
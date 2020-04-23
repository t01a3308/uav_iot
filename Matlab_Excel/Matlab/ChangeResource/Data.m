x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [22.1158	19.2257	17.9129	17.1521	16.6045	16.1478	15.8946	1.57E+01	15.5237	15.2303	15.144]; % global tsp
% y2 = [25.4262	23.3506	22.3279	21.7961	21.4271	21.0466	20.8611	2.07E+01	20.5152	20.2677	20.1852]; % local tsp
y3 = [25.4262	23.3506	22.3279	21.7961	21.4271	21.0466	20.8611	2.07E+01	20.5152	20.2677	20.1852]; % iterative
y4 = [21.0919	19.015	17.7611	17.0957	16.5762	16.2255	15.9578	15.7358	15.5936	15.42	15.3255]; % inter-cell
y5 = [22.698	20.3831	19.489	18.8104	18.2777	17.8941	17.6218	1.74E+01	17.1917	16.8224	16.8272];% GELSGA
y6 = [20.9136	18.8238	17.5235	16.8392	16.2979	15.9052	15.6084	1.54E+01	15.1905	15.0352	14.8931];% CVRP
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
ylabel('Total data (MB)');
legend('Inter-cell','Iterative strategy','Global TSP','GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
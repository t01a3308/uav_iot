x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [22.1158	19.2257	17.9129	17.1521	16.6045	16.1478	15.8946	1.57E+01	15.5237	15.2303	15.144]; % global tsp
% y2 = [25.4262	23.3506	22.3279	21.7961	21.4271	21.0466	20.8611	2.07E+01	20.5152	20.2677	20.1852]; % local tsp
y3 = [25.9619	23.771	22.7589	22.1874	21.8355	21.4398	21.183	21.094	20.8994	20.6241	20.4831]; % iterative
y4 = [22.3825	20.0155	18.7869	18.2053	17.6484	17.2729	17.043	16.761	16.6046	16.4695	16.3167]; % inter-cell
y5 = [22.698	20.3831	19.489	18.8104	18.2777	17.8941	17.6218	1.74E+01	17.1917	16.8224	16.8272];% GELSGA
y6 = [22.0908	19.7764	18.4977	17.8915	17.2764	16.9005	16.596	16.349	16.1491	15.9804	15.8251];% CVRP
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
ylabel('Total data (MB)');
legend('Inter-cell','Iterative strategy','CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
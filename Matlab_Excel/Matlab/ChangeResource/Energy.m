x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [27.0956	20.8903	18.0305	16.4227	15.2418	14.2798	13.7174	13.3919	12.9081	12.3024	12.1195]; % global tsp
% y2 = [27.4452	22.1692	19.5819	18.1969	17.1188	16.2904	15.7171	15.4641	15.0738	14.4565	14.1665]; % local tsp
y3 = [27.5792	22.304	19.7254	18.2648	17.2617	16.4174	15.7451	15.5406	15.1729	14.5869	14.1433]; % iterative
y4 = [25.7077	21.0592	18.4324	17.1904	16.0613	15.2905	14.7265	14.2172	13.8877	13.6033	13.3031]; % inter-cell
y5 = [28.327	23.3728	21.3962	19.9822	18.8091	18.0126	17.4188	16.8569	16.5161	15.7322	15.7303];% GELSGA
y6 = [25.1592	20.5872	17.8833	16.5212	15.3304	14.5084	13.8689	13.3558	12.9501	12.6017	12.2865];% CVRP
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
ylabel('Energy consumption (MJ) ');
legend('Inter-cell','Iterative strategy','CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
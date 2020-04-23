x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [323.596	235.003	194.162	171.209	154.345	140.619	132.538	127.937	121.024	112.33	109.78]; % global tsp
% y2 = [328.185	252.817	215.857	196.072	180.672	168.838	160.648	157.033	151.458	142.639	138.498]; % local tsp
y3 = [328.185	252.817	215.857	196.072	180.672	168.838	160.648	157.033	151.458	142.639	138.498]; % iterative
y4 = [291.384	227.602	188.968	168.514	152.97	141.891	133.432	126.975	122.15	117.348	114.012]; % inter-cell
y5 = [341.308	270.471	242.624	222.054	205.281	193.883	185.345	177.353	172.386	161.19	161.22];% GELSGA
y6 = [286.172	222.148	182.71	161.362	144.884	132.973	123.698	116.706	111.052	105.789	101.867];% CVRP
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
ylabel('Flight distance (km) ');
legend('Inter-cell','Iterative strategy','Global TSP','GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
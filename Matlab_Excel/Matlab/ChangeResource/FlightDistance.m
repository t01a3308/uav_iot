x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [323.596	235.003	194.162	171.209	154.345	140.619	132.538	127.937	121.024	112.33	109.78]; % global tsp
% y2 = [328.185	252.817	215.857	196.072	180.672	168.838	160.648	157.033	151.458	142.639	138.498]; % local tsp
y3 = [330.136	254.779	217.944	197.079	182.748	170.689	161.082	158.162	152.91	144.54	138.202]; % iterative
y4 = [303.404	236.999	199.475	181.734	165.605	154.591	146.536	139.258	134.55	130.487	126.202]; % inter-cell
y5 = [341.308	270.471	242.624	222.054	205.281	193.883	185.345	177.353	172.386	161.19	161.22];% GELSGA
y6 = [295.565	230.256	191.627	172.168	155.161	143.418	134.285	126.948	121.154	116.179	111.673];% CVRP
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
ylabel('Flight distance (km) ');
legend('Inter-cell','Iterative strategy','CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
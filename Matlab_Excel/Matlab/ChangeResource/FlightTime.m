x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [21.0827	16.1608	13.8919	12.6167	11.6798	10.9173	10.4683	10.2127	9.82867	9.34568	9.204]; % global tsp
% y2 = [21.3377	17.1505	15.0972	13.998	13.1425	12.485	12.03	11.8292	11.5195	11.0296	10.7994]; % local tsp
y3 = [21.4442	17.2578	15.2114	14.0522	13.256	12.5861	12.0524	11.8901	11.5983	11.1334	10.7812]; % iterative
y4 = [19.9591	16.2699	14.1853	13.1997	12.3036	11.6917	11.2442	10.8399	10.5784	10.3526	10.1145]; % inter-cell
y5 = [22.0667	18.1313	16.5647	15.4415	14.5097	13.8764	13.4021	12.9581	12.6822	12.0602	12.0618];% GELSGA
y6 = [19.5236	15.8954	13.7493	12.6682	11.7234	11.071	10.5636	10.156	9.8341	9.55772	9.30741];% CVRP
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
ylabel('Flight time (h)');
legend('Inter-cell','Iterative strategy','CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
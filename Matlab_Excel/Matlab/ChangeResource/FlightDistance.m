x = [250 300 350 400 450 500 550 600];
y3 = [209.276	186.998	174.517	160.707	157.701	151.466	142.201	137.193]; % iterative
y4 = [202.582	183.055	166.881	155.073	146.47	133.91	124.739	118.22]; % inter-cell
y5 = [212.351	189.35	173.142	161.713	153	146.173	141.519	137.954];% GELSGA
y6 = [197.325	180.092	164.468	153.648	146.058	138.776	133.491	128.042];% CVRP
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
xlabel('Amount of resource per flight');
ylabel('Flight distance (km) ');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([250 300 350 400 450 500 550 600]);
xlim([250 600]);
hold off;
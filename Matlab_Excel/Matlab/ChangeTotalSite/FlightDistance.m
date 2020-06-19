x = [300 350 400 450 500 550 600 650 700];
y3 = [110.808	118.699	124.433	136.741	151.466	161.624	169.514	177.491	193.766]; % iterative
y4 = [90.1999	99.5025	109.068	119.676	133.91	148.785	159.562	171.861	182.979]; % inter-cell
y5 = [102.073	118.686	125.836	133.507	146.173	162.217	175.632	194.011	211.358];% GELSGA
y6 = [90.4979	101.89	113.396	126.447	138.776	151.461	163.21	178.201	191.965]; %CVRP
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
xlabel('Total sites');
ylabel('Flight distance (km) ');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 350 400 450 500 550 600 650 700]);
xlim([300 700]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
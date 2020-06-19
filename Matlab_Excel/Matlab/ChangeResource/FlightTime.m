x = [250 300 350 400 450 500 550 600];
y3 = [16.5085	15.2315	14.5135	13.7336	13.5638	13.2063	12.6906	12.4054]; % iterative
y4 = [16.0767	14.9627	14.0431	13.3744	12.8874	12.1909	11.6799	11.3166]; % inter-cell
y5 = [16.6926	15.3771	14.4555	13.8073	13.3144	12.9306	12.6698	12.4667];% GELSGA
y6 = [15.8309	14.8488	13.9636	13.3483	12.9226	12.5098	12.2135	11.9034];% CVRP
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
ylabel('Flight time (h)');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([250 300 350 400 450 500 550 600]);
xlim([250 600]);
hold off;
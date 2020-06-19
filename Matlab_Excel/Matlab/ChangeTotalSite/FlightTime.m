x = [300 350 400 450 500 550 600 650 700];
y3 = [9.04309	9.93744	10.7126	11.911	13.2063	14.2453	15.1669	16.1436	17.5807]; % iterative
y4 = [7.8982	8.87065	9.85705	10.9411	12.1909	13.4766	14.5484	15.7647	16.8867]; % inter-cell
y5 = [8.55927	9.94734	10.8008	11.7399	12.9306	14.3088	15.5477	17.1243	18.614];% GELSGA
y6 = [7.91476	9.00428	10.1029	11.3406	12.5098	13.6911	14.8383	16.2259	17.5042]; %CVRP
close all;
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
width = 2;
plot(x,y4,'-kv', 'LineWidth',width);
hold on;
grid on;
plot(x,y3,'-rd', 'LineWidth',width);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',width);
plot(x,y6,'->','Color',[1 0.5 0], 'LineWidth',width);
xlabel('Total sites');
ylabel('Flight time (h)');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 350 400 450 500 550 600 650 700]);
xlim([300 700]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
x = [300 325 350 375 400 425 450 475 500];
y1 = [10.9718	11.8828	12.7293	13.5563	14.4745	15.3624	16.1638	17.0609	17.9129]; % global tsp
y2 = [3.40015	4.38428	5.22632	5.99121	7.12061]; % local tsp
y3 = [14.1817	15.3844	16.3966	17.3721	18.5384	19.5803	20.5995	21.6923	22.7589]; % iterative
y4 = [11.6689	12.6372	13.4811	14.3454	15.2934	16.1303	16.9624	17.9717	18.7869]; % inter-cell
y5 = [11.4303	12.4354	13.4426	14.472	15.4219	16.4161	17.4165	18.4845	19.4848];% GELSGA
y6 = [11.3337	12.3051	13.1476	14.0148	14.9721	15.9383	16.6932	17.6794	18.4977]; %CVRP
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
xlabel('Total sites');
ylabel('Total data (MB)');
legend('Inter-cell','Iterative strategy', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 325 350 375 400 425 450 475 500]);
xlim([300 500]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
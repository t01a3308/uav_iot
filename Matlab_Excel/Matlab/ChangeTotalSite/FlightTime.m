x = [300 325 350 375 400 425 450 475 500];
y1 = [8.71906	9.47039	10.0483	10.6537	11.3627	12.052	12.5888	13.2652	13.8919]; % global tsp
y2 = [3.41307	4.45627	5.33061	6.08049	7.27282]; % local tsp
y3 = [9.88282	10.7049	11.2905	11.8529	12.6111	13.2629	13.8798	14.4977	15.2114]; % iterative
y4 = [9.25007	9.90207	10.5031	11.1047	11.7633	12.3464	12.9393	13.6106	14.1853]; % inter-cell
y5 = [9.49767	10.3811	11.2792	12.1861	12.97	13.8684	14.6991	15.7006	16.5647];% GELSGA
y6 = [8.69793	9.35923	9.97129	10.5852	11.2496	11.8691	12.4812	13.1394	13.7493]; %CVRP
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
ylabel('Flight time (h)');
legend('Inter-cell','Iterative strategy', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 325 350 375 400 425 450 475 500]);
xlim([300 500]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
x = [250 300 350 400 450 500 550 600];
y3 = [62.2964	57.4517	52.3694	49.6978	48.3808	45.7457	45.1383	43.4054]; % iterative
y4 = [50.1149	44.8407	39.6964	35.5496	33.0663	32.3217	30.4742	29.4912]; % inter-cell
y5 = [66.0383	60.7246	57.0624	53.8595	51.6433	50.3875	49.8582	48.2758];% GELSGA
y6 = [61.7986	57.6428	54.0924	50.6063	49.2532	47.678	46.7314	44.7056];% CVRP
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
ylabel('Spanning time (m)');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([250 300 350 400 450 500 550 600]);
xlim([250 600]);
hold off;
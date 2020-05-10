x = [300 325 350 375 400 425 450 475 500];
y1 = [28.1515	30.343	32.4624	34.7294	37.2022	38.9795	41.5602	43.53	45.4731]; % global tsp
y2 = [26.3205	32.6161	38.4725	43.5438	52.3585]; % local tsp
y3 = [33.31493	35.0472	37.3951	39.4898	41.9787	43.8335	46.4377	48.3252	50.3522]; % iterative
y4 = [25.7692	27.1537	28.7107	30.849	33.0651	34.9022	36.6959	38.6099	39.8326]; % inter-cell
y5 = [32.1577	33.6476	38.7133	40.6121	43.7253	46.3521	48.3169	52.6711	54.8049];% GELSGA
y6 = [30.3346	32.0987	34.4535	36.4351	39.0624	41.1208	43.2879	45.1935	46.7781]; %CVRP
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
ylabel('Spanning time (s)');
legend('Inter-cell','Iterative strategy', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 325 350 375 400 425 450 475 500]);
xlim([300 500]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
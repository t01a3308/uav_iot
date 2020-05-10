x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [74.0722	54.4814	45.4731	40.298	36.5986	33.6579	32.2935	30.2481	29.454	28.0379	27.0421]; % global tsp
% y2 = [76.9592	59.0954	49.9981	45.4609	41.9076	39.1179	37.151	35.3785	34.5819	33.3716	31.799]; % local tsp
y3 = [76.889	58.8437	50.3522	45.2369	41.8953	39.0802	37.0946	35.3414	34.6341	33.1981	31.8363]; % iterative
y4 = [62.6625	48.4823	39.8326	36.1634	33.4567	31.7294	30.2873	29.311	28.2699	26.7951	26.3958]; % inter-cell
y5 = [76.6022	61.8094	54.8049	52.0253	49.0548	45.813	44.6076	43.6008	42.7721	40.6771	39.4735];% GELSGA
y6 = [69.0069	55.1699	46.7781	42.8945	39.3924	36.9205	35.3693	33.882	32.6209	31.4498	30.8642];% CVRP
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
ylabel('Spanning time (m)');
legend('Inter-cell','Iterative strategy','CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
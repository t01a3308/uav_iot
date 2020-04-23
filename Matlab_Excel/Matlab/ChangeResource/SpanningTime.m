x = [150 200 250 300 350 400 450 500 550 600 650];
y1 = [74.0722	54.4814	45.4731	40.298	36.5986	33.6579	32.2935	30.2481	29.454	28.0379	27.0421]; % global tsp
% y2 = [76.9592	59.0954	49.9981	45.4609	41.9076	39.1179	37.151	35.3785	34.5819	33.3716	31.799]; % local tsp
y3 = [76.9592	59.0954	49.9981	45.4609	41.9076	39.1179	37.151	35.3785	34.5819	33.3716	31.799]; % iterative
y4 = [60.3379	46.951	38.1556	33.7505	30.0106	26.4386	25.3787	23.4437	23.1385	21.4085	20.4602]; % inter-cell
y5 = [76.6022	61.8094	54.8049	52.0253	49.0548	45.813	44.6076	43.6008	42.7721	40.6771	39.4735];% GELSGA
y6 = [64.2909	51.4898	43.5418	38.7574	35.7926	33.2015	30.6139	29.9652	29.3702	26.7398	26.8817];% CVRP
close all;
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
plot(x,y4,'-kv', 'LineWidth',2);
hold on;
grid on;
% plot(x,y2,'-bs', 'LineWidth',2);
plot(x,y3,'-rd', 'LineWidth',2);
plot(x,y1,'-mo', 'LineWidth',2);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
plot(x,y6,'->','Color',[1 0.5 0], 'LineWidth',2);
xlabel('Amount of resource per flight');
ylabel('Spanning time (m)');
legend('Inter-cell','Iterative strategy','Global TSP','GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([150 200 250 300 350 400 450 500 550 600 650]);
xlim([150 650]);
hold off;
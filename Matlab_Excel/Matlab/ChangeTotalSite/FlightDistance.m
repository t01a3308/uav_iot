x = [300 325 350 375 400 425 450 475 500];
y1 = [123.465	134.17	141.738	149.857	159.778	169.481	176.308	185.611	194.162]; % global tsp
y2 = [30.0745	41.0657	49.4748	54.8183	69.1083]; % local tsp
y3 = [144.377	156.376	164.135	171.466	182.326	191.256	199.56	207.778	217.944]; % iterative
y4 = [132.987	141.925	149.963	157.999	167.067	174.76	182.63	191.811	199.475]; % inter-cell
y5 = [137.48	150.562	163.893	177.44	188.711	202.175	214.292	229.448	242.272];% GELSGA
y6 = [123.049	132.153	140.39	148.648	157.82	166.168	174.385	183.329	191.627]; %CVRP
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
ylabel('Flight distance (km) ');
legend('Inter-cell','Iterative strategy', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 325 350 375 400 425 450 475 500]);
xlim([300 500]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
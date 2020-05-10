x = [300 325 350 375 400 425 450 475 500];
y1 = [11.2893	12.2621	13.0208	13.8127	14.7317	15.6287	16.3375	17.2102	18.0305]; % global tsp
y2 = [4.60093	5.9503	7.17933	8.19474	9.77035]; % local tsp
y3 = [12.7876	13.8515	14.6172	15.3537	16.337	17.1864	17.9918	18.7993	19.7254]; % iterative
y4 = [11.9904	12.8398	13.6249	14.4109	15.2687	16.0314	16.8065	17.6816	18.4324]; % inter-cell
y5 = [12.2691	13.4111	14.5695	15.7396	16.7564	17.9152	18.9904	20.283	21.3962];% GELSGA
y6 = [11.2948	12.1561	12.9553	13.7563	14.6214	15.4302	16.2295	17.0878	17.8833]; %CVRP
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
ylabel('Energy consumption (MJ) ');
legend('Inter-cell','Iterative strategy', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 325 350 375 400 425 450 475 500]);
xlim([300 500]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
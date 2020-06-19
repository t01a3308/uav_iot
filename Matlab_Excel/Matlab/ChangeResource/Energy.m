x = [250 300 350 400 450 500 550 600];
y3 = [21.6804	20.0643	19.1551	18.17	17.9558	17.5033	16.8534	16.4927]; % iterative
y4 = [21.1255	19.7167	18.5539	17.7089	17.0936	16.2007	15.5404	15.1033]; % inter-cell
y5 = [21.9149	20.2506	19.0852	18.266	17.6434	17.1588	16.8297	16.5731];% GELSGA
y6 = [20.8244	19.582	18.4635	17.6859	17.1484	16.6269	16.253	15.8608];% CVRP
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
ylabel('Energy consumption (MJ) ');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
xticks([250 300 350 400 450 500 550 600]);
xlim([250 600]);
hold off;
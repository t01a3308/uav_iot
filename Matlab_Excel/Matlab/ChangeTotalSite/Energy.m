x = [300 350 400 450 500 550 600 650 700];
y3 = [11.9142	13.1232	14.1822	15.785	17.5033	18.8977	20.1464	21.4732	23.38]; % iterative
y4 = [10.4717	11.7788	13.1	14.5369	16.2007	17.9188	19.355	20.9836	22.4878]; % inter-cell
y5 = [11.3049	13.1381	14.2954	15.5709	17.1588	18.9832	20.6334	22.7202	24.6916];% GELSGA
y6 = [10.4922	11.9476	13.4144	15.0662	16.6269	18.2011	19.736	21.5841	23.2871]; %CVRP
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
xlabel('Total sites');
ylabel('Energy consumption (MJ) ');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 350 400 450 500 550 600 650 700]);
xlim([300 700]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
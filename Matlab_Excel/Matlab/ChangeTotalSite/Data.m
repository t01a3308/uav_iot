x = [300 350 400 450 500 550 600 650 700];
y3 = [14.4958	16.4359	18.2072	20.7454	23.0276	25.208	27.3047	29.435	32.0363]; % iterative
y4 = [11.1913	12.7444	14.3499	16.076	17.9787	19.9288	21.7755	24.1982	26.8932]; % inter-cell
y5 = [12.9784	14.8688	15.9377	17.6589	19.9587	22.2136	23.8688	26.0191	28.0853];% GELSGA
y6 = [11.151	12.9829	14.6867	16.7481	18.7109	20.5769	22.4017	24.5622	26.6442]; %CVRP
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
ylabel('Total data (MB)');
legend('Inter-cell','Iterative strategy', 'GELSGA', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 350 400 450 500 550 600 650 700]);
xlim([300 700]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
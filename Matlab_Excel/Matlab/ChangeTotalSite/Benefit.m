x = [40 50 60 70 80];
y1 = [415794	529114	629502	757394	872828]; % global tsp
y2 = [369734	438265	513216	622443	637086]; % local tsp
y3 = [385736	481045	564739	676552	796219]; % iterative
y4 = [387889	509862	597369	7.29E+05	844968]; % inter-cell
y5 = [399984	503843	544887	679711	805780];% GELSGA
close all;
figure1 = figure;
% Create axes
axes1 = axes('Parent',figure1);
plot(x,y1,'-kv', 'LineWidth',2);
hold on;
grid on;
plot(x,y2,'-bs', 'LineWidth',2);
plot(x,y3,'-rd', 'LineWidth',2);
plot(x,y4,'-mo', 'LineWidth',2);
plot(x,y5,'-x','Color',[0 0.4 0], 'LineWidth',2);
xlabel('Total site');
ylabel('Total benefit');
legend('Global TSP','Local TSP','Iterative strategy','Inter-cell','GELSGA');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([40 50 60 70 80]);
xlim([40 80]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
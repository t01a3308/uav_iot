x = [300 325 350 375 400 425 450 475 500];
y1 = [3.96944	4.20701	4.45406	4.71464	4.91223	5.08336	5.32095	5.52342	5.60427]; % global tsp
y2 = [369734	438265	513216	622443	637086]; % local tsp
y3 = [2.44245	2.57718	2.74706	2.92015	3.03637	3.16091	3.285	3.41853	3.49016]; % iterative
y4 = [2.7197	2.91258	3.09216	3.26395	3.41612	3.56579	3.70694	3.83607	3.94556]; % inter-cell
y5 = [3.87915	4.11635	4.32245	4.51009	4.70584	4.85604	5.01516	5.13789	5.18553];% GELSGA
y6 = [2.7092	2.88219	3.04848	3.20048	3.33886	3.46251	3.58741	3.71024	3.80343]; %CVRP
y1 = y1*1000000;
y3 = y3*1000000;
y4 = y4*1000000;
y5 = y5*1000000;
y6 = y6*1000000;
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
ylabel('Total benefit');
legend('Inter-cell','Iterative strategy', 'CVRP');
set(axes1,'XColor',[0 0 0],'YColor',[0 0 0]);
xticks([300 325 350 375 400 425 450 475 500]);
xlim([300 500]);
ytickformat('%,4.4g');
xtickformat('%,4.4g');
hold off;
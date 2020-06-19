m2=importdata('Utility_Iterative.plt',' ',0);
m3=importdata('Utility_Intercell.plt',' ',0);
m4=importdata('Utility_GELSGA.plt',' ',0);
m5=importdata('Utility_CVRP.plt',' ',0);
x2=m2(:,1);
x3=m3(:,1);
x4=m4(:,1);
x5=m5(:,1);
y2=m2(:,2);
y3=m3(:,2);
y4=m4(:,2);
y5=m5(:,2);
q2=trapz(x2,y2);
q3=trapz(x3,y3);
q4=trapz(x4,y4);
q5=trapz(x5,y5);
width = 2;
plot(x3,y3,'-k', 'LineWidth',width);
hold on;
plot(x2,y2,'-r', 'LineWidth',width);
 plot(x4,y4,'Color',[0 0.4 0], 'LineWidth',width);
plot(x5,y5,'Color',[1 0.5 0], 'LineWidth',width);
grid on;
xlabel('Time (s)');
ylabel('Cumulative utility');
legend('Inter-cell','Iterative strategy','GELSGA','CVRP');
hold off;
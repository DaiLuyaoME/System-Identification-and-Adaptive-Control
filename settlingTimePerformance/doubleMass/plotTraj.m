disVal = dis.signals.values;
velVal = vel.signals.values;
accVal = acc.signals.values;
jerkVal = jerk.signals.values;
snapVal = snap.signals.values;
time = dis.time;

figure;

subplot(5,1,1);
plot(time,disVal*1000,'DisplayName','λ��','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('λ��/mm','FontSize',10);

subplot(5,1,2);
plot(time,velVal*1000,'DisplayName','�ٶ�','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('�ٶ�/(mm/s)','FontSize',10);

subplot(5,1,3);
plot(time,accVal,'DisplayName','���ٶ�','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('���ٶ�/(m/s^2)','FontSize',10);

subplot(5,1,4);
plot(time,jerkVal,'DisplayName','�Ӽ��ٶ�','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('�Ӽ��ٶ�/(m/s^3)','FontSize',10);

subplot(5,1,5);
plot(time,snapVal,'DisplayName','�ӼӼ��ٶ�','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('�Ӽ��ٶ�/(m/s^4)','FontSize',10);
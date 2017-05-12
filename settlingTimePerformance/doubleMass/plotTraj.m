disVal = dis.signals.values;
velVal = vel.signals.values;
accVal = acc.signals.values;
jerkVal = jerk.signals.values;
snapVal = snap.signals.values;
time = dis.time;

figure;

subplot(5,1,1);
plot(time,disVal*1000,'DisplayName','位移','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('位移/mm','FontSize',10);

subplot(5,1,2);
plot(time,velVal*1000,'DisplayName','速度','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('速度/(mm/s)','FontSize',10);

subplot(5,1,3);
plot(time,accVal,'DisplayName','加速度','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('加速度/(m/s^2)','FontSize',10);

subplot(5,1,4);
plot(time,jerkVal,'DisplayName','加加速度','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('加加速度/(m/s^3)','FontSize',10);

subplot(5,1,5);
plot(time,snapVal,'DisplayName','加加加速度','LineWidth',2);
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',10);
ylabel('加加速度/(m/s^4)','FontSize',10);
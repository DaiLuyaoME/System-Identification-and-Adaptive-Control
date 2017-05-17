close all;
figure1 = figure;

% 创建 axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% 创建 plot
h=plot(pidErr.time,pidErr.signals.values,'DisplayName','PID Error','LineWidth',2);

% 创建 xlabel
xlabel('$t/s$','Interpreter','latex','FontSize',20);

% 创建 title
title('控制误差');

% 创建 ylabel
ylabel('$e\left( t \right)$','Interpreter','latex','FontSize',20);

box(axes1,'on');
% 设置其余坐标轴属性
set(axes1,'FontSize',16);

hold on;

temp1=max(abs(pidErr.signals.values));
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;

plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);

% 创建 legend
legend1 = legend(gca,'show');
%% 绘制误差和加速度轨迹
close all;
disName={'PID Error','leadErr'};


plotError(Err.time,Err.signals.values*1e9,'控制误差');
hold on;
plotError(noise.time,noise.signals.values*1e9,'噪声');
temp1=max(abs(Err.signals.values*1e9));
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;
plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);

legend(gca,'show');

%% 绘制指定时间段误差和加速度轨迹
close;
time=Err.time;
index=time>0.043 & time < 0.053;
plotError(time(index),Err.signals.values(index)*1e9,'控制误差');
hold on;

temp1=max(abs(Err.signals.values*1e9));
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;
plot(time(index),ratio*acc.signals.values(index),'DisplayName','Acceleration','LineWidth',2);
plot(time(index),noise.signals.values(index)*1e9,'DisplayName','Noise','LineWidth',2);
legend(gca,'show');
axis tight;

%%
close all;
temp = zpk(sysc);
kp=dcgain(temp);
kd=kp/abs(temp.Z{1});
accErr=m0*acc.signals.values/kp;
jerkErr=-m0*kd*jerk.signals.values/(kp*kp);
plot(acc.time,accErr,'LineWidth',2);
hold on;
plot(acc.time,Err.signals.values,'LineWidth',2);
plot(acc.time,jerkErr,'LineWidth',2);
hold on;
plot(acc.time,Err.signals.values-accErr,'LineWidth',2);
plot(acc.time,Err.signals.values-accErr-jerkErr,'LineWidth',2);
%%
switch controllerFlag
    case 1
        Err=pidErr;
    case 2
        Err=leadErr;
end
close all;
temp = zpk(sysc);
kp=dcgain(temp);
kd=kp/abs(temp.Z{1});
accErr=m0*acc.signals.values/kp;
plot(acc.time,accErr,'LineWidth',2);
hold on;
plot(acc.time,Err.signals.values,'LineWidth',2);



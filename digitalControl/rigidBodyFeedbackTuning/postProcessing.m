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
switch controllerFlag
    case 1
        Err=pidErr;
    case 2
        Err=leadErr;
end

plotError(Err.time,Err.signals.values,disName{controllerFlag});
hold on;

temp1=max(abs(Err.signals.values));
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;
plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
legend(gca,'show');

% temp1=max(abs(pidErr.signals.values));
% % temp2=max(abs(feedbackForce.signals.values));
% % ratio=0.5*temp1/temp2;
% plot(feedbackForce.time,ratio*feedbackForce.signals.values,'DisplayName','Feedback Force','LineWidth',2);
% legend(gca,'show');
%% 计算建立时间
len=numel(acc.time);
err=Err.signals.values;
tol=2e-8;
settlingTime=inf;
for i = 1:len
    if ( abs(err(i)) < tol )
        if(mean(err(i:(i+100))) <tol)
            settlingTime=Err.time(i)
            break;
        end
    end
end

%% 对比两种反馈控制器下的跟踪误差
close all;
disName={'PID Error','Lead Error'};
plotError(pidErr.time,pidErr.signals.values,disName{1});
hold on;
plotError(leadErr.time,leadErr.signals.values,disName{2});
hold on;
temp1=max(abs(leadErr.signals.values));
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;
plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
legend(gca,'show');

%%
hold on;

temp1=max(abs(Err.signals.values));
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;
plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
legend(gca,'show');
%% 绘制估计质量与加速度的对比关系
close;
temp1=max(abs(esMass.data));
temp2=max(abs(acc.signals.values));
ratio=0.2*temp1/temp2;
plot(acc.time,20+ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
hold on;
plot(esMass.time,esMass.data,'DisplayName','esMass','LineWidth',2);
legend(gca,'show');
%% 绘制不同加速度前馈系数下的误差对比图
close;
plot(err23.time,err23.signals.values,'LineWidth',2);
hold on;
plot(err23_5.time,err23_5.signals.values,'LineWidth',2);
hold on;
plot(err24.time,err24.signals.values,'LineWidth',2);
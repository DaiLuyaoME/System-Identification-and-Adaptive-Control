%% 这份误差绘制程序是为新项目的跟踪控制仿真准备的

disVal=dis.signals.values;
accVal=acc.signals.values;
time=dis.time;

close all;
disName={'PID Error','leadErr'};
switch controllerFlag
    case 1
        Err=pidErr;
    case 2
        Err=leadErr;
end

plotError(Err.time,Err.signals.values*1e9,disName{controllerFlag});
hold on;

temp1=max(abs(Err.signals.values*1e9));
temp2=max(abs(accVal));
temp3=max(abs(disVal));

ratio12=0.5*temp1/temp2;
ratio13=temp1/temp3;

plot(time,ratio12*accVal,'DisplayName','加速度（缩放后）','LineWidth',2);
plot(time,ratio13*disVal,'DisplayName','位移（缩放后）','LineWidth',2);
legend(gca,'show');
%% 这份误差绘制程序是为新项目的跟踪控制仿真准备的

disVal=dis.signals.values;
accVal=acc.signals.values;
time=dis.time;
index=time>0.035;
close all;
disName={'PID Error','leadErr'};
switch controllerFlag
    case 1
        Err=pidErr;
    case 2
        Err=leadErr;
end

plotError(Err.time(index),Err.signals.values(index)*1e9,'控制误差');
hold on;

temp1=max(abs(Err.signals.values*1e9));
temp2=max(abs(accVal));
temp3=max(abs(disVal));

ratio12=0.5*temp1/temp2;
ratio13=temp1/temp3;

plot(time(index),ratio12*accVal(index),'DisplayName','加速度（缩放后）','LineWidth',2);
% plot(time(index),ratio13*disVal(index),'DisplayName','位移（缩放后）','LineWidth',2);
legend(gca,'show');
xlim([0.04,0.05]);
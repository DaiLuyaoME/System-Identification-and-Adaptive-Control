accData=acc.signals.values;
switch controllerFlag
    case 1
        errData=pidErr.signals.values;
    case 2
        errData=leadErr.signals.values;
end
tempAcc=max(abs(accData));
tempErr1=max(abs(errData));

%% 跑第二遍时执行
accData=acc.signals.values;
switch controllerFlag
    case 1
        errData=pidErr.signals.values;
    case 2
        errData=leadErr.signals.values;
end
tempErr2=max(abs(errData));

%% 计算估计的前馈系数
mass=23+tempErr1/(tempErr1-tempErr2)

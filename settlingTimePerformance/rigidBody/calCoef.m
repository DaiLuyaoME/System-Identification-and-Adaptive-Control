accData=acc.signals.values;
switch controllerFlag
    case 1
        errData=pidErr.signals.values;
    case 2
        errData=leadErr.signals.values;
end
tempAcc=max(abs(accData));
tempErr1=max(abs(errData));

%% �ܵڶ���ʱִ��
accData=acc.signals.values;
switch controllerFlag
    case 1
        errData=pidErr.signals.values;
    case 2
        errData=leadErr.signals.values;
end
tempErr2=max(abs(errData));

%% ������Ƶ�ǰ��ϵ��
mass=23+tempErr1/(tempErr1-tempErr2)

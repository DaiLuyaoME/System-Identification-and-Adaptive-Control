%% ��������Ƴ�����Ϊ����Ŀ�ĸ��ٿ��Ʒ���׼����

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

plot(time,ratio12*accVal,'DisplayName','���ٶȣ����ź�','LineWidth',2);
plot(time,ratio13*disVal,'DisplayName','λ�ƣ����ź�','LineWidth',2);
legend(gca,'show');
%% ��������Ƴ�����Ϊ����Ŀ�ĸ��ٿ��Ʒ���׼����

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

plotError(Err.time(index),Err.signals.values(index)*1e9,'�������');
hold on;

temp1=max(abs(Err.signals.values*1e9));
temp2=max(abs(accVal));
temp3=max(abs(disVal));

ratio12=0.5*temp1/temp2;
ratio13=temp1/temp3;

plot(time(index),ratio12*accVal(index),'DisplayName','���ٶȣ����ź�','LineWidth',2);
% plot(time(index),ratio13*disVal(index),'DisplayName','λ�ƣ����ź�','LineWidth',2);
legend(gca,'show');
xlim([0.04,0.05]);
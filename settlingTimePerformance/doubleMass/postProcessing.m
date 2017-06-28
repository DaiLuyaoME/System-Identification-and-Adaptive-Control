close all;
figure1 = figure;

% ���� axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% ���� plot
h=plot(pidErr.time,pidErr.signals.values,'DisplayName','PID Error','LineWidth',2);

% ���� xlabel
xlabel('$t/s$','Interpreter','latex','FontSize',20);

% ���� title
title('�������');

% ���� ylabel
ylabel('$e\left( t \right)$','Interpreter','latex','FontSize',20);

box(axes1,'on');
% ������������������
set(axes1,'FontSize',16);

hold on;

temp1=max(abs(pidErr.signals.values));
temp2=max(abs(acc.signals.values));
ratioAcc=0.5*temp1/temp2;

plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);

% ���� legend
legend1 = legend(gca,'show');
%% �������ͼ��ٶȹ켣
close all;
disName={'�������',' ',' ','�������'};
switch controllerFlag
    case 1
        Err=pidErr;
    case 4
        Err=leadErr;
end

plotError(Err.time,Err.signals.values*1e9,disName{controllerFlag});
hold on;

temp1=max(abs(Err.signals.values*1e9));
temp2=max(abs(acc.signals.values));
ratioAcc=0.5*temp1/temp2;
plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','���ź�ļ��ٶ���״','LineWidth',2);
plot(noise.time,noise.signals.values*1e9,'DisplayName','����','LineWidth',2);
grid on;
legend(gca,'show');

% temp1=max(abs(pidErr.signals.values));
% % temp2=max(abs(feedbackForce.signals.values));
% % ratio=0.5*temp1/temp2;
% plot(feedbackForce.time,ratio*feedbackForce.signals.values,'DisplayName','Feedback Force','LineWidth',2);
% legend(gca,'show');
%% ������� ���ٶȹ켣 snap
close all;
disName={'PID Error',' ',' ','leadErr'};
switch controllerFlag
    case 1
        Err=pidErr;
    case 4
        Err=leadErr;
end

plotError(Err.time,Err.signals.values,disName{controllerFlag});
hold on;
% plot(noise.time,noise.signals.values,'DisplayName','noise','LineWidth',2);
temp1=max(abs(Err.signals.values));
temp2=max(abs(acc.signals.values));
temp3=max(abs(snap.signals.values));
ratioAcc=0.5*temp1/temp2;
ratioSnap=temp1/temp3;
plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','scaled acceleration','LineWidth',3);
plot(snap.time,ratioSnap*snap.signals.values,'DisplayName','scaled snap','LineWidth',3);
legend(gca,'show');

% temp1=max(abs(pidErr.signals.values));
% % temp2=max(abs(feedbackForce.signals.values));
% % ratio=0.5*temp1/temp2;
% plot(feedbackForce.time,ratio*feedbackForce.signals.values,'DisplayName','Feedback Force','LineWidth',2);
% legend(gca,'show');
%% plot snap acc dis
close all;
plot(dis.time,dis.signals.values,'DisplayName','trajectory','LineWidth',2);
hold on;
temp1=max(abs(dis.signals.values));
temp2=max(abs(acc.signals.values));
temp3=max(abs(snap.signals.values));
ratioAcc=0.5*temp1/temp2;
ratioSnap=0.5*temp1/temp3;
plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','scaled acceleration','LineWidth',2);
plot(snap.time,ratioSnap*snap.signals.values,'DisplayName','scaled snap','LineWidth',2);
legend(gca,'show');
xlabel('$Time\left( s \right)$','Interpreter','latex','FontSize',20);

ylabel('$r\left( t \right)\left( m \right)$','Interpreter','latex','FontSize',20);

%% ���㽨��ʱ��
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

%% �Ա����ַ����������µĸ������
close all;
disName={'PID Error','Lead Error'};
plotError(pidErr.time,pidErr.signals.values,disName{1});
hold on;
plotError(leadErr.time,leadErr.signals.values,disName{2});
hold on;
temp1=max(abs(leadErr.signals.values));
temp2=max(abs(acc.signals.values));
ratioAcc=0.5*temp1/temp2;
plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
legend(gca,'show');

%%
hold on;

temp1=max(abs(Err.signals.values));
temp2=max(abs(acc.signals.values));
ratioAcc=0.5*temp1/temp2;
plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
legend(gca,'show');
%% ���ƹ�����������ٶȵĶԱȹ�ϵ
close;
temp1=max(abs(esMass.data));
temp2=max(abs(acc.signals.values));
ratioAcc=0.2*temp1/temp2;
plot(acc.time,20+ratioAcc*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
hold on;
plot(esMass.time,esMass.data,'DisplayName','esMass','LineWidth',2);
legend(gca,'show');
%% ���Ʋ�ͬ���ٶ�ǰ��ϵ���µ����Ա�ͼ
close;
plot(err23.time,err23.signals.values,'LineWidth',2);
hold on;
plot(err23_5.time,err23_5.signals.values,'LineWidth',2);
hold on;
plot(err24.time,err24.signals.values,'LineWidth',2);

%%
temperror=Err.signals.values(Err.time >0.1 & Err.time < 0.2);
errorDist=abs(mean(temperror));
disturbance=dcgain(sysc)*errorDist
%%
close all;
h=bodeplot(sysc);
setoptions(h,'FreqUnits','Hz');
%%
close all;
h=bodeplot(sysc*Gp);
setoptions(h,'FreqUnits','Hz');
%% ���ƶ�λ���
close all;
switch controllerFlag
    case 1
        Err=pidErr;
    case 4
        Err=leadErr;
end

plotError(Err.time,Err.signals.values*1e9,'��λ���');
hold on;

% temp1=max(abs(Err.signals.values*1e9));
% temp2=max(abs(acc.signals.values));
% ratioAcc=0.5*temp1/temp2;
% plot(acc.time,ratioAcc*acc.signals.values,'DisplayName','���ź�ļ��ٶ���״','LineWidth',2);
legend(gca,'show');

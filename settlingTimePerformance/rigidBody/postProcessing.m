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
ratio=0.5*temp1/temp2;

plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);

% ���� legend
legend1 = legend(gca,'show');
%% �������ͼ��ٶȹ켣
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
temp2=max(abs(acc.signals.values));
ratio=0.5*temp1/temp2;
plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
legend(gca,'show');

% temp1=max(abs(pidErr.signals.values));
% % temp2=max(abs(feedbackForce.signals.values));
% % ratio=0.5*temp1/temp2;
% plot(feedbackForce.time,ratio*feedbackForce.signals.values,'DisplayName','Feedback Force','LineWidth',2);
% legend(gca,'show');
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
%% ���ƹ�����������ٶȵĶԱȹ�ϵ
close;
temp1=max(abs(esMass.data));
temp2=max(abs(acc.signals.values));
ratio=0.2*temp1/temp2;
plot(acc.time,20+ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
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



%% ����ʱ�����
figure;
if type == 3 && controllerFlag == 2
    temp = zpk(sysc);
    kp=dcgain(temp);
    kd=kp/abs(temp.Z{1});
end

global t1;
global t2;
global jerkBound;

s=tf('s');
Ge=kd/kp*(1-2*exp(-t1*s)+exp(-t2*s))/kp;
step(-800*Ge);
figure;
impulse(800*Ge);
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



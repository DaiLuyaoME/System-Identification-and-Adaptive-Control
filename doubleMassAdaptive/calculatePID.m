m=25;
fbw=100; % desired bandwidth/Hz
alpha=3; % ratio
kp=m*(2*pi*fbw)^2/alpha; %proportional gain
fi=fbw/alpha^2;% integrator frequency
fd=fbw/alpha;% differential frequency
flp=alpha*fbw; % low pass frequency
wlp=flp*2*pi;
zlp=0.707; % low pass requency damping ratio

%pid控制器
num1=[1 2*pi*fi];
den1=[1 0];

num2=[1 2*pi*fd];
den2=[2*pi*fd];

syspi=tf(num1,den1);
syspd=tf(num2,den2);
syspid=kp*series(syspi,syspd);

%二阶低通滤波器
num3=wlp*wlp;
den3=[1 2*wlp*zlp wlp*wlp];
syslp=tf(num3,den3);

%% 计算控制系统的实际系数值
clc;
num=syspid.Numerator;
num=num{1};
den=syspid.Denominator;
den=den{1};
den=den(2);

KI=40;
KV=0.2;

Kp=num(2)/den;
Kp=Kp/(KI*KV);
Kp=Kp/1e9

Kd=num(1)/den;
Kd=Kd/(KI*KV);
Kd=Kd/1e9

Ki=num(3)/den;
Ki=Ki/(KI*KV);
Ki=Ki/1e9
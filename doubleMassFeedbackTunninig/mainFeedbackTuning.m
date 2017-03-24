clear all;
close all;
clc;
%% 被控双质量块传递函数模型
m1=20;
m2=5;
fn=200;
wn=fn*2*pi;
k=wn*wn*m1*m2/(m1+m2);
numGp=k;
denGp=[m1*m2 0 (m1+m2)*k 0 0];
Gp=tf(numGp,denGp);


%% Bulter那篇文章给出的PID+低通滤波的控制器设计
m=m1+m2; % mass/kg
fbw=40; % desired bandwidth/Hz
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


%% 自己设计的超前滞后控制器
load leadController.mat
sysLead=leadController*m;
fb=100;
wb=fb*2*pi;
w=wb*3;
sysLowPassForLead=lowPassFilter(w,0.7);

%% 控制方案选择

flag=1;

switch flag
    case 1 %采用pid
        % 完整的反馈控制器
        sysc=series(syspid,syslp);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 2 %采用超前
        sysc=series(sysLead,sysLowPassForLead);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
end

%% 其他系统参数定义
%白噪声模块用的采用频率，需要大于系统的运行频率
fs=10000;
Ts=1/fs;
sigma = 0e-9;%噪声的标准差，单位m
varNoise=sigma*sigma;


%%
feedbackTuning;
set_param('feedbackTuning/feedback controller','Numerator','numerator');
set_param('feedbackTuning/feedback controller','Denominator','denominator');
sim('feedbackTuning',[0 0.26]);

%%
GpGc=series(Gp,sysc);
rltool(GpGc);

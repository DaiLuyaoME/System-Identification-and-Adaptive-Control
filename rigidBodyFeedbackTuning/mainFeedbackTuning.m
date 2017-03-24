clear all;
close all;
clc;
%% 被控刚体传递函数模型
m=25;
num=[1];
den=[m 0 0];
Gp=tf(num,den);
numRigidBody=num;
denRigidBody=den;

%% Bulter那篇文章给出的PID+低通滤波的控制器设计
fbw=200; % desired bandwidth/Hz
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
sysLead=leadController;


%% 控制方案选择

controllerFlag=1;

switch controllerFlag
    case 1 %采用pid
        % 完整的反馈控制器
        sysc=1.1187*series(syspid,syslp);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 2 %采用超前
        sysc=sysLead;
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
end
GpGc=series(Gp,sysc);
%% 其他系统参数定义
%白噪声模块用的采用频率，需要大于系统的运行频率
fs=10000;
Ts=1/fs;
sigma = 10e-9;%噪声的标准差，单位m
varNoise=sigma*sigma;


%%
feedbackTuning;

set_param('feedbackTuning/feedback controller','Numerator','numerator');
set_param('feedbackTuning/feedback controller','Denominator','denominator');
set_param('feedbackTuning/rigid body','Numerator','numRigidBody');
set_param('feedbackTuning/rigid body','Denominator','denRigidBody');
sim('feedbackTuning',[0 0.6]);

%% 开环传递函数

% rltool(GpGc);

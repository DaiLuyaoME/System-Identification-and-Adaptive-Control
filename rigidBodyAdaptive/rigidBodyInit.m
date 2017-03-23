clear all;
close all;
clc;
global esMass;

esMass=24;%初始估计质量
m0=25;%真实质量
fs=5000;%采样频率
Ts=1/fs;%采样周期

%% 反馈控制器初始化
m=esMass;
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

controllerFlag=2;

switch controllerFlag
    case 1 %采用pid
        % 完整的反馈控制器
        sysc=series(syspid,syslp);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 2 %采用超前
        sysc=sysLead;
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
end
% GpGc=series(Gp,sysc);

%% 测量噪声
%白噪声模块用的采用频率，需要大于系统的运行频率
sigma = 5e-9;%噪声的标准差，单位m
varNoise=sigma*sigma;%注意，白噪声的模块中的Noise Power 需要填成varNoise*Ts
noisePower=varNoise*Ts;


%% Qfilter
fq=6000;
wq=fq*2*pi;
numQ=wq*wq;
denQ=[1 2*wq*0.707 wq*wq];
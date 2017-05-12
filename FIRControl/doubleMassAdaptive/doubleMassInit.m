clear all;
close all;
clc;
%%
global coef; %前馈控制系数
coef=zeros(6,1);
global m1;
global m2;
global k;

m1=20;%台子质量
m2=5; %驱动器质量
fn=1500; %双质量块模型共振频率
wn=fn*2*pi;
k=wn*wn*m1*m2/(m1+m2);

fs=5000;%采样频率
Ts=1/fs;%采样周期

% 前馈系数初始估计值
global m11;
global m22;
m11=m1-1;
m22=m2-0.5;
% coef(1)=m11*m22/k; % snap 前馈系数
% coef(2)=m11+m22; % 加速度前馈系数

%% 被控双质量块传递函数模型
numGp=k;
denGp=[m1*m2 0 (m1+m2)*k 0 0];
Gp=tf(numGp,denGp);


%% Bulter那篇文章给出的PID+低通滤波的控制器设计
m=m1+m2; % mass/kg
fbw=80; % desired bandwidth/Hz
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
% sysLead=leadController;
%% 自己设计的pid控制器
load pidController.mat
myPID=pidController;
%%
load lead400.mat
%% 控制方案选择

controllerFlag=4;

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
    case 3 %采用自己的PID
        sysc=myPID;
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 4 %共振频率为400Hz时用的超前控制器
        sysc=tf(lead400);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
end
discSysc=c2d(sysc,Ts,'tustin');
numGc=discSysc.Numerator; numGc=numGc{1};
denGc=discSysc.Denominator; denGc=denGc{1};
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
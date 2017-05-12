clear all;
close all;
clc;
global esMass;

esMass=23;%初始估计质量
m0=25;%真实质量
fs=5000;%采样频率
Ts=1/fs;%采样周期
%% 刚体模型
numGp = [1];
denGp = [m0 0 0];
Gp=tf(numGp,denGp);
%% 反馈控制器初始化
m=esMass;
fbw=120; % desired bandwidth/Hz
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
type=1;
switch type
    case 1
        load leadController.mat
        sysLead=leadController;
    case 2
        load Lead.mat;
        sysLead=m0*Lead;
    case 3
        load PD3.mat;
        sysLead=tf(m0*PD);
    case 4
        load PID.mat;
        sysLead=m0*PID;
    case 5
        load tempFeedbackController.mat;
        sysLead=m0*tf(tempFeedbackController);
end

%% 控制方案选择

controllerFlag=2;

switch controllerFlag
    case 1 %采用pid
        % 完整的反馈控制器
        sysc=series(syspid,syslp);
        numGc=sysc.Numerator;numGc=numGc{1};
        denGc=sysc.Denominator;denGc=denGc{1};
    case 2 %采用超前
        sysc=sysLead;
        numGc=sysc.Numerator;numGc=numGc{1};
        denGc=sysc.Denominator;denGc=denGc{1};
end
% GpGc=series(Gp,sysc);

%% 测量噪声
%白噪声模块用的采用频率，需要大于系统的运行频率
TsN=1e-6;
sigma = 2.5e-9;%噪声的标准差，单位m
varNoise=sigma*sigma;%注意，白噪声的模块中的Noise Power 需要填成varNoise*Ts
noisePower=varNoise*TsN;


%% Qfilter
fq=6000;
wq=fq*2*pi;
numQ=wq*wq;
denQ=[1 2*wq*0.707 wq*wq];
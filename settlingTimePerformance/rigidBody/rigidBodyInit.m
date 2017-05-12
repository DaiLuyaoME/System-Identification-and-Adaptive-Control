clear all;
close all;
clc;
global esMass;

esMass=23;%��ʼ��������
m0=25;%��ʵ����
fs=5000;%����Ƶ��
Ts=1/fs;%��������
%% ����ģ��
numGp = [1];
denGp = [m0 0 0];
Gp=tf(numGp,denGp);
%% ������������ʼ��
m=esMass;
fbw=120; % desired bandwidth/Hz
alpha=3; % ratio
kp=m*(2*pi*fbw)^2/alpha; %proportional gain
fi=fbw/alpha^2;% integrator frequency
fd=fbw/alpha;% differential frequency
flp=alpha*fbw; % low pass frequency
wlp=flp*2*pi;
zlp=0.707; % low pass requency damping ratio

%pid������
num1=[1 2*pi*fi];
den1=[1 0];

num2=[1 2*pi*fd];
den2=[2*pi*fd];

syspi=tf(num1,den1);
syspd=tf(num2,den2);
syspid=kp*series(syspi,syspd);

%���׵�ͨ�˲���
num3=wlp*wlp;
den3=[1 2*wlp*zlp wlp*wlp];
syslp=tf(num3,den3);

%% �Լ���Ƶĳ�ǰ�ͺ������
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

%% ���Ʒ���ѡ��

controllerFlag=2;

switch controllerFlag
    case 1 %����pid
        % �����ķ���������
        sysc=series(syspid,syslp);
        numGc=sysc.Numerator;numGc=numGc{1};
        denGc=sysc.Denominator;denGc=denGc{1};
    case 2 %���ó�ǰ
        sysc=sysLead;
        numGc=sysc.Numerator;numGc=numGc{1};
        denGc=sysc.Denominator;denGc=denGc{1};
end
% GpGc=series(Gp,sysc);

%% ��������
%������ģ���õĲ���Ƶ�ʣ���Ҫ����ϵͳ������Ƶ��
TsN=1e-6;
sigma = 2.5e-9;%�����ı�׼���λm
varNoise=sigma*sigma;%ע�⣬��������ģ���е�Noise Power ��Ҫ���varNoise*Ts
noisePower=varNoise*TsN;


%% Qfilter
fq=6000;
wq=fq*2*pi;
numQ=wq*wq;
denQ=[1 2*wq*0.707 wq*wq];
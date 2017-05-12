clear all;
close all;
clc;
global esMass;
global coef;
coef=zeros(3,1);

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
fbw=200; % desired bandwidth/Hz
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

load Lead.mat;
sysLead=m0*Lead;


%% ���Ʒ���ѡ��

controllerFlag=2;

switch controllerFlag
    case 1 %����pid
        % �����ķ���������
        sysc=series(syspid,syslp);
    case 2 %���ó�ǰ
        sysc=sysLead;
end
sysc=c2d(sysc,Ts,'tustin');
numGc=sysc.Numerator;numGc=numGc{1};
denGc=sysc.Denominator;denGc=denGc{1};
% GpGc=series(Gp,sysc);

%% ��������
%������ģ���õĲ���Ƶ�ʣ���Ҫ����ϵͳ������Ƶ��
sigma = 5e-9;%�����ı�׼���λm
varNoise=sigma*sigma;%ע�⣬��������ģ���е�Noise Power ��Ҫ���varNoise*Ts
noisePower=varNoise*Ts;


%% Qfilter
fq=6000;
wq=fq*2*pi;
numQ=wq*wq;
denQ=[1 2*wq*0.707 wq*wq];
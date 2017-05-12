clear all;
close all;
clc;
%%
global coef; %ǰ������ϵ��
coef=zeros(6,1);
global m1;
global m2;
global k;

m1=20;%̨������
m2=5; %����������
fn=1500; %˫������ģ�͹���Ƶ��
wn=fn*2*pi;
k=wn*wn*m1*m2/(m1+m2);

fs=5000;%����Ƶ��
Ts=1/fs;%��������

% ǰ��ϵ����ʼ����ֵ
global m11;
global m22;
m11=m1-1;
m22=m2-0.5;
% coef(1)=m11*m22/k; % snap ǰ��ϵ��
% coef(2)=m11+m22; % ���ٶ�ǰ��ϵ��

%% ����˫�����鴫�ݺ���ģ��
numGp=k;
denGp=[m1*m2 0 (m1+m2)*k 0 0];
Gp=tf(numGp,denGp);


%% Bulter��ƪ���¸�����PID+��ͨ�˲��Ŀ��������
m=m1+m2; % mass/kg
fbw=80; % desired bandwidth/Hz
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
load leadController.mat
% sysLead=leadController;
%% �Լ���Ƶ�pid������
load pidController.mat
myPID=pidController;
%%
load lead400.mat
%% ���Ʒ���ѡ��

controllerFlag=4;

switch controllerFlag
    case 1 %����pid
        % �����ķ���������
        sysc=series(syspid,syslp);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 2 %���ó�ǰ
        sysc=sysLead;
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 3 %�����Լ���PID
        sysc=myPID;
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 4 %����Ƶ��Ϊ400Hzʱ�õĳ�ǰ������
        sysc=tf(lead400);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
end
discSysc=c2d(sysc,Ts,'tustin');
numGc=discSysc.Numerator; numGc=numGc{1};
denGc=discSysc.Denominator; denGc=denGc{1};
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
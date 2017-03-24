clear all;
close all;
clc;
%% ���ظ��崫�ݺ���ģ��
m=25;
num=[1];
den=[m 0 0];
Gp=tf(num,den);
numRigidBody=num;
denRigidBody=den;

%% Bulter��ƪ���¸�����PID+��ͨ�˲��Ŀ��������
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
load leadController.mat
sysLead=leadController;


%% ���Ʒ���ѡ��

controllerFlag=1;

switch controllerFlag
    case 1 %����pid
        % �����ķ���������
        sysc=1.1187*series(syspid,syslp);
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
    case 2 %���ó�ǰ
        sysc=sysLead;
        numerator=sysc.Numerator;numerator=numerator{1};
        denominator=sysc.Denominator;denominator=denominator{1};
end
GpGc=series(Gp,sysc);
%% ����ϵͳ��������
%������ģ���õĲ���Ƶ�ʣ���Ҫ����ϵͳ������Ƶ��
fs=10000;
Ts=1/fs;
sigma = 10e-9;%�����ı�׼���λm
varNoise=sigma*sigma;


%%
feedbackTuning;

set_param('feedbackTuning/feedback controller','Numerator','numerator');
set_param('feedbackTuning/feedback controller','Denominator','denominator');
set_param('feedbackTuning/rigid body','Numerator','numRigidBody');
set_param('feedbackTuning/rigid body','Denominator','denRigidBody');
sim('feedbackTuning',[0 0.6]);

%% �������ݺ���

% rltool(GpGc);

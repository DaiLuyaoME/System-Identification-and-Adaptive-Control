% clear all;
close all;
clc;
%%
fs=5000;%����Ƶ��
Ts=1/fs;%��������

%% ���ɱ��ض���ģ��
% flag == 1 ����ģ��
% flag == 2 ˫������ģ��
modelType=2;
switch modelType
    case 1
        mass=25;
        totalMass=mass;
        fr=0;
        Gp = createPlantModel(mass,fr,modelType);
                load pd.mat;
    case 2
        mass=[20,5];
        totalMass=sum(mass);
        fr=700;
        Gp = createPlantModel(mass,fr,modelType);
                load pdDoubleMass.mat;
                pd=pdDoubleMass;
    case 3
        totalMass=25;
        fr=[700,1000];
        Gp=createPlantModel(totalMass,fr,modelType);
%         load pdTripleMass.mat;
%         pd=pdTripleMass;
end
[numGp,denGp] = tfdata(Gp,'v');
GpDis = c2d(Gp,Ts,'zoh');
[numGpDis,denGpDis] = tfdata(GpDis,'v');
%% �������������
controllerFlag=2;
% Gc = totalMass*createFeedbackController(controllerFlag);
Gc = totalMass*tf(pd);
GcDis = c2d(Gc,Ts,'tustin');
[numGc,denGc] = tfdata(Gc,'v');
[numGcDis,denGcDis] = tfdata(GcDis,'v');
%% ǰ�����������
% temp=dcgain(GpDis);
F1=numGpDis/sum(numGpDis);
F2=denGpDis/sum(numGpDis);
%% ����Simulinkģ�͵Ĵ��ݺ���
% mode: 1 ����ģʽ��2 ��ɢģʽ
mode=2;
switch mode
    case 1
        replace_block('main/feedback controller','DiscreteTransferFcn','TransferFcn','noprompt');
        set_param('main/feedback controller/feedbackController','Denominator','denGc');
        set_param('main/feedback controller/feedbackController','Numerator','numGc');
        
        replace_block('main/plant model','DiscreteTransferFcn','TransferFcn','noprompt');
        set_param('main/plant model/plantModel','Denominator','denGp');
        set_param('main/plant model/plantModel','Numerator','numGp');
        
    case 2
        replace_block('main/feedback controller','TransferFcn','DiscreteTransferFcn','noprompt');
        set_param('main/feedback controller/feedbackController','Denominator','denGcDis');
        set_param('main/feedback controller/feedbackController','Numerator','numGcDis');
        set_param('main/feedback controller/feedbackController','SampleTime','Ts');
        %         replace_block('main/plant model','TransferFcn','DiscreteTransferFcn','noprompt');
        %         set_param('main/plant model/plantModel','Denominator','denGpDis');
        %         set_param('main/plant model/plantModel','Numerator','numGpDis');
        %
end
%% Fir�˲�����IIR�˲�����ʼ��
a=1/(Ts*Ts);
firCoef=[1 -2 1]*a;
temp=20*tf([1,0 0],1);
temp=c2d(temp,Ts,'tustin');
[numIIR,denIIR]=tfdata(temp,'v');
%% ��������
%������ģ���õĲ���Ƶ�ʣ���Ҫ����ϵͳ������Ƶ��
TsN=1e-5;
sigma = 10e-9;%�����ı�׼���λm
varNoise=sigma*sigma;%ע�⣬��������ģ���е�Noise Power ��Ҫ���varNoise*Ts
noisePower=varNoise*TsN;

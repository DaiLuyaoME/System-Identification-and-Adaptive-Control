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
    case 2
        mass=[20,5];
        totalMass=sum(mass);
        fr=700;
        Gp = createPlantModel(mass,fr,modelType);
end
[numGp,denGp] = tfdata(Gp,'v');
%% �������������
controllerFlag=2;
Gc = totalMass*createFeedbackController(controllerFlag);
Gc = totalMass*tf(pd);
[numGc,denGc] = tfdata(Gc,'v');
%% ��������
%������ģ���õĲ���Ƶ�ʣ���Ҫ����ϵͳ������Ƶ��
TsN=1e-5;
sigma = 10e-9;%�����ı�׼���λm
varNoise=sigma*sigma;%ע�⣬��������ģ���е�Noise Power ��Ҫ���varNoise*Ts
noisePower=varNoise*TsN;

% clear all;
close all;
clc;
%%
fs=5000;%采样频率
Ts=1/fs;%采样周期

%% 生成被控对象模型
% flag == 1 刚体模型
% flag == 2 双质量块模型
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
%% 反馈控制器设计
controllerFlag=2;
% Gc = totalMass*createFeedbackController(controllerFlag);
Gc = totalMass*tf(pd);
GcDis = c2d(Gc,Ts,'tustin');
[numGc,denGc] = tfdata(Gc,'v');
[numGcDis,denGcDis] = tfdata(GcDis,'v');
%% 前馈控制器设计
% temp=dcgain(GpDis);
F1=numGpDis/sum(numGpDis);
F2=denGpDis/sum(numGpDis);
%% 设置Simulink模型的传递函数
% mode: 1 连续模式；2 离散模式
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
%% Fir滤波器和IIR滤波器初始化
a=1/(Ts*Ts);
firCoef=[1 -2 1]*a;
temp=20*tf([1,0 0],1);
temp=c2d(temp,Ts,'tustin');
[numIIR,denIIR]=tfdata(temp,'v');
%% 测量噪声
%白噪声模块用的采用频率，需要大于系统的运行频率
TsN=1e-5;
sigma = 10e-9;%噪声的标准差，单位m
varNoise=sigma*sigma;%注意，白噪声的模块中的Noise Power 需要填成varNoise*Ts
noisePower=varNoise*TsN;

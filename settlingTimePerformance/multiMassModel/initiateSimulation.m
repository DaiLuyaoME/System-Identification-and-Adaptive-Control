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
    case 2
        mass=[20,5];
        totalMass=sum(mass);
        fr=700;
        Gp = createPlantModel(mass,fr,modelType);
end
[numGp,denGp] = tfdata(Gp,'v');
%% 反馈控制器设计
controllerFlag=2;
Gc = totalMass*createFeedbackController(controllerFlag);
Gc = totalMass*tf(pd);
[numGc,denGc] = tfdata(Gc,'v');
%% 测量噪声
%白噪声模块用的采用频率，需要大于系统的运行频率
TsN=1e-5;
sigma = 10e-9;%噪声的标准差，单位m
varNoise=sigma*sigma;%注意，白噪声的模块中的Noise Power 需要填成varNoise*Ts
noisePower=varNoise*TsN;

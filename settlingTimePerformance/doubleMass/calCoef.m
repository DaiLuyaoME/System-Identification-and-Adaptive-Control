%% 学习加速前馈系数
tempErr=leadErr.signals.values;
tempAcc=acc.signals.values;
tempErr=tempErr(leadErr.time>0.044 & leadErr.time<0.048);
tempAcc=tempAcc(leadErr.time>0.044 & leadErr.time<0.048);
mf=24.5;
mass=mf+mean(tempErr)*dcgain(sysc)/mean(tempAcc)

%% 学习snap前馈系数
tempErr=leadErr.signals.values;
tempSnap=snap.signals.values;
tempErr=tempErr(leadErr.time>0.016 & leadErr.time<0.0205);
tempSnap=tempSnap(leadErr.time>0.016 & leadErr.time<0.0205);
snapCoef=mean(tempErr)*dcgain(sysc)/mean(tempSnap)
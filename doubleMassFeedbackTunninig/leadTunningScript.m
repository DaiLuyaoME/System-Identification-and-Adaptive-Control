

fb=100;
wb=fb*2*pi;
w=3*wb;
sysfilter=lowPassFilter(w,0.7);
G=series(Gp,sysLowPassForLead);

%%
close all;
step(feedback(series(Gp,sysc),1));

%%
leadController=tf(C);
save leadController.mat leadController;

%%
nyquist(syslp*syspid*Gp)
%%
xlim([-100 3]);
num=[1];
den=[1 0 0];
sysG=tf(num,den);

fb=200;
wb=fb*2*pi;
w=3*wb;
sysfilter=lowPassFilter(w,0.7);
G=series(sysG,sysfilter);

%%
close all;
step(feedback(series(Gp,sysc),1));

%%
leadController=tf(C);
save leadController.mat leadController;

%%
close all;
nyquist(sysc*Gp);
xlim([-3 3]);
%%
close all;
pzmap(1+sysc*Gp);

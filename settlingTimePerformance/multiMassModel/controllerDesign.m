%%
Gp=tf(1,[1 0 0]);
%%
f=700;
w=f*2*pi;
z=0.01;
num=1;den=[1 0 0];Gp1=tf(num,den);
num=w*w;den=[1 2*z*w w*w];Gp2=tf(num,den);
Gp=Gp1*Gp2;
%%
sisotool(Gp);
%%
leadDoubleMass1 = tf(pd);
save leadDoubleMass1.mat leadDoubleMass1;
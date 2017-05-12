%%
Gp=tf(1,[1 0 0]);
%%
sisotool(Gp);
%%
leadlag1 = tf(pd);
save leadlag1.mat leadlag1;
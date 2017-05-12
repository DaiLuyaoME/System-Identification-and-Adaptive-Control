G=tf(1,[1 0 0]);
sisotool(G);
%% ±£´æ¿ØÖÆÆ÷
tempFeedbackController = C;
save tempFeedbackController.mat tempFeedbackController;

%%
a=10;
phim=asind( (1-1/a)/ (1+1/a) )
fb=80;
wb=fb*2*pi;
z=0.5*wb/sqrt(a)
p=a*z
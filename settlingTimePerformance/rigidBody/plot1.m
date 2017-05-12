m=40;
% rigid body mode
num=1;
den=m*[1 0 0];
G0=tf(num,den);

% firt resonant mode
f1=600;
w1=f1*2*pi;
z1=0.01;
num=-1;
den=m*[1,2*z1*w1,w1*w1];
G1=tf(num,den);

% second resonant mode
f2=1000;
w2=f2*2*pi;
z2=0.01;
num=1;
den=m*[1,2*z2*w2,w2*w2];
G2=tf(num,den);

% third resonant mode
f3=2000;
w3=f3*2*pi;
z3=0.01;
num=-1;
den=m*[1,2*z3*w3,w3*w3];
G3=tf(num,den);

% plant model
G=G0+G1+G2+G3;

% plot bode diagram
close all;
h=bodeplot(G,{100,100000});
setoptions(h,'FreqUnits','Hz');

%%
% triple mass model
m1 = 20;
m2 = 5;
m3 = 5;

f1=700;
w1=f1*2*pi;
k1=w1*w1*m1*m2/(m1+m2);

f2=1000;
w2=f2*2*pi;
k2=w2*w2*m2*m3/(m2+m3);

A= [ ...
    0 1 0 0 0 0
    -k1/m1 0 k1/m1 0 0 0
    0 0 0 1 0 0
    k1/m2 0 -(k1+k2)/m2 0 k2/m2 0
    0 0 0 0 0 1
    0 0 k2/m3 0 -k2/m3 0 ...
    ];
B= [ ...
    0 0 0
    1/m1 0 0
    0 0 0
    0 1/m2 0
    0 0 0
    0 0 1/m3 ...d
    ]
% B= [0 1/m1 0 0 0 0]';
C= [ 0 0 0 0 1 0];
D=0;
G=tf(ss(A,B,C,D));
close;
h=bodeplot(G);
setoptions(h,'FreqUnits','Hz');
%%
% triple mass model,transfer function
m1 = 20;
m2 = 5;
m3 = 5;
m=m1+m2+m3;
f1=700;
w1=f1*2*pi;
k1=w1*w1*m1*m2/(m1+m2);

f2=1000;
w2=f2*2*pi;
k2=w2*w2*m2*m3/(m2+m3);

num =k1*k2;
den = [m 0 2*m*k1+2*m*k2 0 3*m*k1*k2 0 0];
G=tf(num,den);
close;
h=bodeplot(G,{100,50000});
setoptions(h,'FreqUnits','Hz');

%%
% triple mass model state space with damping
m1 = 20;
m2 = 5;
m3 = 5;

f1=700;
w1=f1*2*pi;
k1=w1*w1*m1*m2/(m1+m2);
c1=2*0.003*w1*(m1+m2);

f2=1000;
w2=f2*2*pi;
k2=w2*w2*m2*m3/(m2+m3);
c2=2*0.003*w2*(m2+m3);

A= [ ...
    0 1 0 0 0 0
    -k1/m1 -c1/m1 k1/m1 c1/m1 0 0
    0 0 0 1 0 0
    k1/m2 c1/m1 -(k1+k2)/m2 -(c1+c2)/m2 k2/m2 c2/m2
    0 0 0 0 0 1
    0 0 k2/m3 c2/m3 -k2/m3 -c2/m3 ...
    ];
B= [ ...
    0 0 0
    1/m1 0 0
    0 0 0
    0 1/m2 0
    0 0 0
    0 0 1/m3 ...
    ]
B= [0 1/m1 0 0 0 0]';
C= [ 0 0 0 0 1 0];
D=0;
% G=tf(ss(A,B,C,D));
G=ss(A,B,C,D);
close;
h=bodeplot(G,{100,10000});
setoptions(h,'FreqUnits','Hz');
%%
% double mass model
m1 = 20;
m2 = 5;

f1=700;
w1=f1*2*pi;
k1=w1*w1*m1*m2/(m1+m2);


A= [ ...
    0 1 0 0;
    -k1/m1 0 k1/m1 0;
    0 0 1 0;
    k1/m2 0 -k1/m2 0;
    ];
B= [0 1/m1 0 0]';
C= [0 0 1 0];
D=0;
G=tf(ss(A,B,C,D));
close;
h=bodeplot(G);
setoptions(h,'FreqUnits','Hz');
%% ´®Áª·½Ê½
m=30;
% rigid body mode
num=1;
den=m*[1 0 0];
G0=tf(num,den);

% firt resonant mode
f1=700;
w1=f1*2*pi;
z1=0.01;
num=w1*w1;
den=[1,2*z1*w1,w1*w1];
G1=tf(num,den);

% second resonant mode
f2=1000;
w2=f2*2*pi;
z2=0.01;
num=w2*w2;
den=[1,2*z2*w2,w2*w2];
G2=tf(num,den);

% third resonant mode
f3=2000;
w3=f3*2*pi;
z3=0.01;
num=-1;
den=m*[1,2*z3*w3,w3*w3];
G3=tf(num,den);

% plant model
G=G0*G1*G2;

% plot bode diagram
close all;
h=bodeplot(G,{100,100000});
setoptions(h,'FreqUnits','Hz');

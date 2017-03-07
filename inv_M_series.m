% clear all;
% close all;
% signal length
L=600;
%initial value of m series
x1=1;x2=1;x3=0;x4=0;
%square wave signal
S=1;
M=zeros(L,1);
IM=zeros(L,1);
% M series : x(k)=x(k-3) xor x (k-4)
% IM(k) = M(k) xor S(k);
for k=1:L
    M(k) = xor(x3,x4);
    IM(k) = xor(M(k),S);
    if IM(k)==0
        IM(k)=-1;
    end
    x4=x3;x3=x2;x2=x1;x1=M(k);
    S=not(S);
end
subplot(2,1,1)
% plot(M);
stairs(M);
axis([0 L -0.5 1.5]);
xlabel('k');title('M series');
subplot(2,1,2)
% plot(u);
stairs(IM);
xlabel('k');title('inverse M series');
axis([0 L -1.5 1.5]);
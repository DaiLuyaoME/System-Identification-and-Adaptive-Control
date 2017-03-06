a=[1 -1.5 0.7]';
b=[1 0.5]';
d=3;
na=numel(a)-1;
nb=numel(b)-1;
%data length 
uk=zeros(d+nb,1);
yk=zeros(na,1);
% noise series
sigma=sqrt(1);
xi=sigma*randn(L,1);
theta=[a(2:na+1);b];
phi=zeros(L,na+nb+1);
y=zeros(L,1);
for k=1:L
    phi(k,:)=[-yk;uk(d:d+nb)]';
    y(k)=phi(k,:)*theta+xi(k);
    uk=circshift(uk,1);
    uk(1)=IM(k);
    yk=circshift(yk,1);
    yk(1)=y(k);
end
esmated_theta=inv((phi'*phi))*phi'*y;
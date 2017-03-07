a=[1 -1.5 0.7]';
b=[1 0.5]';
d=3;
na=numel(a)-1;
nb=numel(b)-1;
%data length
uk=zeros(d+nb,1);
yk=zeros(na,1);
% noise series
sigma=sqrt(0.1);
xi=sigma*randn(L,1);
theta=[a(2:na+1);b];
phi=zeros(L,na+nb+1);
y=zeros(L,1);

%
% L=100;

n0=( numel(a)+numel(b) )+1;

for k=1:n0
    phi(k,:)=[-yk;uk(d:d+nb)]';
    y(k)=phi(k,:)*theta+xi(k);
    uk=circshift(uk,1);
    uk(1)=IM(k);
    yk=circshift(yk,1);
    yk(1)=y(k);
end
P0=inv( (phi'*phi) );
theta0=P0*phi'*y;
theta_es=theta0';
for k=(n0+1):L
    phi(k,:)=[-yk;uk(d:d+nb)]';
    y(k)=phi(k,:)*theta+xi(k);
    uk=circshift(uk,1);
    uk(1)=IM(k);
    yk=circshift(yk,1);
    yk(1)=y(k);
    
    phi_k=phi(k,:)';
    Rk=P0*phi_k./(1+phi_k'*P0*phi_k);
    theta_k=theta0+Rk*(y(k)-phi_k'*theta0);
    theta_es=[theta_es;theta_k'];
    Pk=( eye(numel(a)+numel(b)-1) -Rk*phi_k' )*P0;
    P0=Pk;
    theta0=theta_k;
end

%% error analysis
close all;
error_a2=theta_es(:,1)-a(2);
plot(abs(error_a2));
hold on;
error_a3=theta_es(:,2)-a(3);
plot(abs(error_a3));
    
    

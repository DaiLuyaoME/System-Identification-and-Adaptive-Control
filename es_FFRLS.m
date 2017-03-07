a=[1 -1.5 0.7]';
b=[1 0.5]';
d=3;
na=numel(a)-1;
nb=numel(b)-1;
%data length
uk=zeros(d+nb,1);
yk=zeros(na,1);
% noise series
sigma=sqrt(0);
xi=sigma*randn(L,1);
theta=[a(2:na+1);b];
phi=zeros(L,na+nb+1);
y=zeros(L,1);
%forgetting factor
lambda=0.99;

%% calculation initiation
n0=( numel(a)+numel(b) )+4;

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

%% recursive calculation
for k=(n0+1):floor(L/2)
    phi(k,:)=[-yk;uk(d:d+nb)]';
    y(k)=phi(k,:)*theta+xi(k);
    uk=circshift(uk,1);
    uk(1)=IM(k);
    yk=circshift(yk,1);
    yk(1)=y(k);
    
    phi_k=phi(k,:)';
    Rk=P0*phi_k./(lambda+phi_k'*P0*phi_k);
    theta_k=theta0+Rk*(y(k)-phi_k'*theta0);
    theta_es=[theta_es;theta_k'];
    Pk=( eye(numel(a)+numel(b)-1) -Rk*phi_k' )*P0./lambda;
    P0=Pk;
    theta0=theta_k;
end

theta=theta*0.1+theta;
for k=ceil(L/2):L
    phi(k,:)=[-yk;uk(d:d+nb)]';
    y(k)=phi(k,:)*theta+xi(k);
    uk=circshift(uk,1);
    uk(1)=IM(k);
    yk=circshift(yk,1);
    yk(1)=y(k);
    
    phi_k=phi(k,:)';
    Rk=P0*phi_k./(lambda+phi_k'*P0*phi_k);
    theta_k=theta0+Rk*(y(k)-phi_k'*theta0);
    theta_es=[theta_es;theta_k'];
    Pk=( eye(numel(a)+numel(b)-1) -Rk*phi_k' )*P0./lambda;
    P0=Pk;
    theta0=theta_k;
end

%% error analysis
% close all;
% error_a1=theta_es(:,1)-theta(2);
% plot(abs(error_a1));
% hold on;
% error_a2=theta_es(:,2)-a(3);
% plot(abs(error_a2));
% 
% ylabel('absolute error');
% xlabel('k');

%%
close all;
figure1 = figure;
axes1 = axes('Parent',figure1);
hold(axes1,'on');
a1_es=theta_es(:,1);
plot(a1_es,'Marker','.','LineWidth',1','DisplayName','${{\hat{a}}_{1}}$');
hold on;
a1=[ones(floor(L/2),1)*a(2);ones(L-floor(L/2),1)*theta(1)];
plot(a1,'DisplayName','${{{a}}_{1}}$');
xlabel('k','FontSize',20,'Interpreter','latex');
ylabel('$\left| {{e}_{k}} \right|$','FontSize',20,'Interpreter','latex');
% a2_es=theta_es(:,2);
% plot(a2_es);
% a2=[ones(floor(L/2),1)*a(3);ones(L-floor(L/2),1)*theta(2)];
% plot(a2);


title('$\lambda =0.40$','FontSize',16,'Interpreter','latex');
box(axes1,'on');
% ´´½¨ legend
legend1 = legend(axes1,'show');
set(legend1,'Interpreter','latex','FontSize',20);
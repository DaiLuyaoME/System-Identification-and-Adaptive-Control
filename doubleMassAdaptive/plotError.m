function plotError(time,data,label)

% ���� plot
h=plot(time,data,'DisplayName',label,'LineWidth',2);
% h.DisplayName=label;
% h.LineWidth=2;

% ���� xlabel
xlabel('$t/s$','Interpreter','latex','FontSize',20);

% ���� title
title('�������');

% ���� ylabel
ylabel('$e\left( t \right)$','Interpreter','latex','FontSize',20);

% ������������������
set(gca,'FontSize',16);
% legend(gca,'show');
% hold on;

end
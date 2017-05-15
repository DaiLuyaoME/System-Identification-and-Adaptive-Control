main;
initiateSimulation;
choice={'step','2nd order traj','3rd order traj','4th order traj','zero'};

% 测试所有轨迹
for i=1:5
    set_param('main/traj','BlockChoice',choice{i});
    if(i==5)
        set_param('main/disturbance','Value','1.5');
    else
        set_param('main/disturbance','Value','0');
    end
    
    sim('main',[0 0.4]);
    
    % 绘制加速度轨迹和误差
    figure;
    plotError(Err.time,Err.signals.values*1e9,'控制误差');
    hold on;
    temp1=max(abs(Err.signals.values*1e9));
    temp2=max(abs(acc.signals.values));
    ratio=0.5*temp1/temp2;
    plot(acc.time,ratio*acc.signals.values,'DisplayName','Acceleration','LineWidth',2);
    legend(gca,'show');
    
    if(i==5)
        title('定位误差');
        time=Err.time;
        plot(time,noise.signals.values*1e9,'DisplayName','Noise','LineWidth',2);
        legend(gca,'show');
        figure
        index=time>0.1 & time < 0.2;
        plotError(time(index),Err.signals.values(index)*1e9,'控制误差');
        hold on;
        
        temp1=max(abs(Err.signals.values*1e9));
        temp2=max(abs(acc.signals.values));
        ratio=0.5*temp1/temp2;
        plot(time(index),ratio*acc.signals.values(index),'DisplayName','Acceleration','LineWidth',2);
        plot(time(index),noise.signals.values(index)*1e9,'DisplayName','Noise','LineWidth',2);
        legend(gca,'show');
        title('定位误差（截取部分时间）');
    else
        title(choice{i});
    end
    
    % % 绘制指定时间段误差和加速度轨迹
    % figure;
    % time=Err.time;
    % index=time>0.1 & time < 0.2;
    % plotError(time(index),Err.signals.values(index)*1e9,'控制误差');
    % hold on;
    % temp1=max(abs(Err.signals.values*1e9));
    % temp2=max(abs(acc.signals.values));
    % ratio=0.5*temp1/temp2;
    % plot(time(index),ratio*acc.signals.values(index),'DisplayName','Acceleration','LineWidth',2);
    % plot(time(index),noise.signals.values(index)*1e9,'DisplayName','Noise','LineWidth',2);
    % legend(gca,'show');
    % title(choice{i});
end






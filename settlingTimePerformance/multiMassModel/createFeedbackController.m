function Gc = createFeedbackController(flag)

switch flag
    case 1 % pid¿ØÖÆÆ÷
        
        
        fbw=120; % desired bandwidth/Hz
        alpha=3; % ratio
        kp=(2*pi*fbw)^2/alpha; %proportional gain
        fi=fbw/alpha^2;% integrator frequency
        fd=fbw/alpha;% differential frequency
        flp=alpha*fbw; % low pass frequency
        wlp=flp*2*pi;
        zlp=0.707; % low pass requency damping ratio
        
        %pid¿ØÖÆÆ÷
        num1=[1 2*pi*fi];
        den1=[1 0];
        
        num2=[1 2*pi*fd];
        den2=[2*pi*fd];
        
        syspi=tf(num1,den1);
        syspd=tf(num2,den2);
        syspid=kp*series(syspi,syspd);
        
        %¶þ½×µÍÍ¨ÂË²¨Æ÷
        num3=wlp*wlp;
        den3=[1 2*wlp*zlp wlp*wlp];
        syslp=tf(num3,den3);
        Gc = syspid*syslp;
    case 2
        load pd.mat;
        Gc=pd;
    case 3
        load PID.mat;
        Gc=PID;
    case 4
        load leadTwoMass.mat;
        Gc=leadTwoMass;
    case 5
        load pidTwoMass.mat;
        Gc=pidTwoMass;
        
end
end
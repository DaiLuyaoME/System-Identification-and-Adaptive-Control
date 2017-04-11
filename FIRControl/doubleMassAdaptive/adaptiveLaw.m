function [sys,x0,str,ts,simStateCompliance] = adaptiveLaw(t,x,u,flag,lam)

switch flag
    
    case 0
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
        
    case 1
        sys=mdlDerivatives(t,x,u);
        
        %   case 2
        %     sys=mdlUpdate(t,x,u);
        
    case 3
        sys=mdlOutputs(t,x,u);
        
        %   case 4
        %     sys=mdlGetTimeOfNextVarHit(t,x,u);
        
    case 9
        sys=mdlTerminate(t,x,u);
    case {2,4}
        sys=[];
        
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
        
end


    function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
        
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 6;
        sizes.NumInputs      = 8;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;   % at least one sample time is needed
        
        sys = simsizes(sizes);
        
        x0  = [];
        
        str = [];
        
        ts  = [0 0];
        
        simStateCompliance = 'UnknownSimState';
        global lambda;
        lambda=lam;
    end
% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
    function sys=mdlDerivatives(t,x,u)
        
        sys = [];
    end

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
% function sys=mdlUpdate(t,x,u)
%
% sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
    function sys=mdlOutputs(t,x,u)
        %u(1)=snap_acc
        %u(2)=jerk_acc
        %u(3)=acc_acc
        %u(4)=vel_acc
        %u(5)=Fc
        %u(6)=err
        
        global P;
        global R;
        global dataMatrix;
        global dataVector;
        global lambda;
        global coef;
        global k;
        global constantAccStartTime;
        global constantAccEndTime;
        global constantJerkStartTime;
        global constantJerkEndTime;
        phi=u(1:6);
        y=u(7);
        %         if ( t>constantJerkStartTime && t < constantJerkEndTime || t>constantAccStartTime && t< constantAccEndTime )
        %         if (  t>constantAccStartTime && t< constantAccEndTime )
        t1=0.015;t2=0.023;
        t3=0.254;t4=0.264;
        %         if (  t> t1 && t< t2 || t > t3 && t < t4 )
        if (  t>constantAccStartTime && t< constantAccEndTime )
            if(numel(dataVector) == 20)
                P=inv(dataMatrix'*dataMatrix);
                coef=P*dataMatrix'*dataVector;
                dataMatrix=[dataMatrix;phi'];
                dataVector=[dataVector;y];
            elseif(numel(dataVector)>20)
                R=P*phi/(lambda+phi'*P*phi);
                coef=coef+R*(y-phi'*coef)/lambda;
                P=(eye(numel(phi))-R*phi')*P/lambda;
            else
                if(abs(u(end))>1e-8)
                    dataMatrix=[dataMatrix;phi'];
                    dataVector=[dataVector;y];
                end
            end
        end
        for i=1:6
            sys(i)=coef(i);
        end
    end

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
% function sys=mdlGetTimeOfNextVarHit(t,x,u)
%
% sampleTime = 1;    %  Example, set the next hit to be one second later.
% sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
    function sys=mdlTerminate(t,x,u)
        global dataMatrix;
        global dataVector;
        %         dataMatrix=[];
        %         dataVector=[];
        sys = [];
    end

% end mdlTerminate
end

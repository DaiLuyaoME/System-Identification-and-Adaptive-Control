function [sys,x0,str,ts,simStateCompliance] = adaptiveLawWithConstraint(t,x,u,flag)

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
        global firOrder;
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = firOrder;
        sizes.NumInputs      = firOrder+4;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;   % at least one sample time is needed
        
        sys = simsizes(sizes);
        
        x0  = [];
        
        str = [];
        
        ts  = [-1 0];
        
        simStateCompliance = 'UnknownSimState';
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
        global P;
        global R;
        global dataMatrix;
        global dataVector;
        global lambda;
        global firOrder;
        global adaptiveFIRCoef;
        global accStartTime;
        global accEndTime;
        global dccStartTime;
        global dccEndTime;
        lambda=0.98;
        sizeBound=20;
        phi=u(1:firOrder);
        y=u(firOrder+1);
        adaptiveFIRCoef(2)=adaptiveFIRCoef(1)/2;
        %         if (  abs(u(end))>3e-7 && t> accStartTime && t< accEndTime || t>dccStartTime && t<dccEndTime)
        %             if(t>0.0037&&t<0.009|| t>0.028&&t<0.034||t>0.103 && t<0.108||t>0.128&&t<0.135)
        if (  abs(u(end))>3e-7 && t> accStartTime && t< 0.024)
            
            if(numel(dataVector) == sizeBound)
                P=inv(dataMatrix'*dataMatrix);
                adaptiveFIRCoef=P*dataMatrix'*dataVector;
                dataMatrix=[dataMatrix;phi'];
                dataVector=[dataVector;y];
            elseif(numel(dataVector)>sizeBound)
                R=P*phi/(lambda+phi'*P*phi);
                adaptiveFIRCoef=adaptiveFIRCoef+R*(y-phi'*adaptiveFIRCoef)/lambda;
                P=(eye(numel(phi))-R*phi')*P/lambda;
            else
                dataMatrix=[dataMatrix;phi'];
                dataVector=[dataVector;y];
            end
        end
        adaptiveFIRCoef(2)=adaptiveFIRCoef(1)/2;
        for i=1:firOrder
            sys(i) = adaptiveFIRCoef(i);
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

end
function sys=mdlTerminate(t,x,u)
global dataMatrix;
global dataVector;
% dataMatrix=[];
% dataVector=[];
sys = [];

end

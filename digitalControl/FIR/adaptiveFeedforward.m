function [sys,x0,str,ts,simStateCompliance] = adaptiveFeedforward(t,x,u,flag)

switch flag
    
    case 0
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes();
        
    case 1
        sys=mdlDerivatives(t,x,u);
        
        %   case 2
        %     sys=mdlUpdate(t,x,u);
        
    case 3
        sys=mdlOutputs(t,x,u);
        %
        %   case 4
        %     sys=mdlGetTimeOfNextVarHit(t,x,u);
        
    case 9
        sys=mdlTerminate(t,x,u);
    case {2,4}
        sys=[];
        
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
        
end


    function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes()
        global firOrder;
        sizes = simsizes;
        sizes.NumContStates  = 0;
        sizes.NumDiscStates  = 0;
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = firOrder;
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
        %     fprintf('count is %d \n',count);
        % u(1)=snap
        % u(2)=jerk;
        % u(3)=acc;
        % u(4)=vel;
        
        global adaptiveFIRCoef;
        % sys=coef'*temp;
        sys=u'*adaptiveFIRCoef;
        % count=count+1;
        
        
        
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

sys = [];

end mdlTerminate
end
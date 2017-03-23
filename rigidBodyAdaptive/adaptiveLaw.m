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
        sizes.NumOutputs     = 1;
        sizes.NumInputs      = 3;
        sizes.DirFeedthrough = 1;
        sizes.NumSampleTimes = 1;   % at least one sample time is needed
        
        sys = simsizes(sizes);
        
        x0  = [];
        
        str = [];
        
        ts  = [0 0];
        
        simStateCompliance = 'UnknownSimState';
        global lambda;
        lambda=lam;
        global esMass;
        fprintf('the initial esmass is %f\n ',esMass);
        %         disp(esMass);
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
        global esMass;
        phi=u(1);
        y=u(2);
        %         if (t<0.036 )
        
        global accEndTime;
        global decStartTime;
        global decEndTime;
        if(t<accEndTime*0.9 && t>accEndTime*0.05 || t > decStartTime && t < decEndTime*0.98)
            
            if(numel(dataVector) == 20)
                P=inv(dataMatrix'*dataMatrix);
                esMass=P*dataMatrix'*dataVector;
                dataMatrix=[dataMatrix;phi];
                dataVector=[dataVector;y];
            elseif(numel(dataVector)>20)
                R=P*phi/(lambda+phi'*P*phi);
                esMass=esMass+R*(y-phi'*esMass)/lambda;
                P=(eye(numel(phi))-R*phi')*P/lambda;
            else
                if(abs(u(3))>4e-8)
                    dataMatrix=[dataMatrix;phi];
                    dataVector=[dataVector;y];
                end
            end
            
        end
        
        sys = esMass;
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
        dataMatrix=[];
        dataVector=[];
        sys = [];
    end

% end mdlTerminate
end

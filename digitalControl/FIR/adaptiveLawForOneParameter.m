function [sys,x0,str,ts,simStateCompliance] = adaptiveLaw(t,x,u,flag)

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
        sizes.NumInputs      = firOrder+3;
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
        global accP;global jerkP;global accR;global jerkR;global dataMatrixAcc; global dataVectorAcc;
        global dataMatrixJerk;global dataVectorJerk; global lambda;
        global firOrder; global adaptiveFIRCoef;  global accStartTime;
        global accConstantAccStartTime; global accConstantAccEndTime; global dccEndTime;
        global firstAccConstantJerkStartTime;global firstAccConstantJerkEndTime;
        global secondAccConstantJerkStartTime;global secondAccConstantJerkEndTime;
        adaptiveMode=0;
        sizeBound=20;
        if( t>accConstantAccStartTime && t<accConstantAccEndTime)
            adaptiveMode=1;%学习加速度前馈系数
        elseif(t>firstAccConstantJerkStartTime&&t<firstAccConstantJerkEndTime||t>secondAccConstantJerkStartTime&&t<secondAccConstantJerkEndTime)
            adaptiveMode=2;%学习jerk前馈系数
        else
            adaptiveMode=0;
        end
        lambda=1;
        switch adaptiveMode
            case 1
                phi=u(adaptiveMode);
                y=u(end-2)-u(end);
                if(numel(dataVectorAcc) == sizeBound)
                    accP=inv(dataMatrixAcc'*dataMatrixAcc);
                    adaptiveFIRCoef(adaptiveMode)=accP*dataMatrixAcc'*dataVectorAcc;
                    dataMatrixAcc=[dataMatrixAcc;phi'];
                    dataVectorAcc=[dataVectorAcc;y];
                elseif(numel(dataVectorAcc)>sizeBound)
                    accR=accP*phi/(lambda+phi'*accP*phi);
                    adaptiveFIRCoef(adaptiveMode)=adaptiveFIRCoef(adaptiveMode)+accR*(y-phi'*adaptiveFIRCoef(adaptiveMode))/lambda;
                    accP=(eye(numel(phi))-accR*phi')*accP/lambda;
                else
                    dataMatrixAcc=[dataMatrixAcc;phi'];
                    dataVectorAcc=[dataVectorAcc;y];
                end
            case 2
                phi=u(adaptiveMode);
                y=u(end-2)-u(end-1);
                if(numel(dataVectorJerk) == sizeBound)
                    jerkP=inv(dataMatrixJerk'*dataMatrixJerk);
                    adaptiveFIRCoef(adaptiveMode)=jerkP*dataMatrixJerk'*dataVectorJerk;
                    dataMatrixJerk=[dataMatrixJerk;phi'];
                    dataVectorJerk=[dataVectorJerk;y];
                elseif(numel(dataVectorJerk)>sizeBound)
                    jerkR=jerkP*phi/(lambda+phi'*jerkP*phi);
                    adaptiveFIRCoef(adaptiveMode)=adaptiveFIRCoef(adaptiveMode)+jerkR*(y-phi'*adaptiveFIRCoef(adaptiveMode))/lambda;
                    jerkP=(eye(numel(phi))-jerkR*phi')*jerkP/lambda;
                else
                    dataMatrixJerk=[dataMatrixJerk;phi'];
                    dataVectorJerk=[dataVectorJerk;y];
                end
        end
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

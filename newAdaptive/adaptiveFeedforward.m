function [sys,x0,str,ts,simStateCompliance] = adaptiveFeedforward(t,x,u,flag,paraNum)
switch flag
    
    case 0
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
        
    case 1
        sys=mdlDerivatives(t,x,u);
        
    case 2
        sys=mdlUpdate(t,x,u);
        
    case 3
        sys=mdlOutputs(t,x,u,paraNum);
        
    case 4
        sys=mdlGetTimeOfNextVarHit(t,x,u);
        
    case 9
        sys=mdlTerminate(t,x,u);
        
    otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
        
end


function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 1;
sizes.NumInputs      = 5;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

x0  = [];

str = [];

ts  = [0 0];

simStateCompliance = 'UnknownSimState';
%%

global dataMatrix;
global count;
global theta_es;
global force;
global P;
global R;
global lambda;
count=0;
theta_es=zeros(2,1);
lambda=1;


% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u,paraNum)
global dataMatrix;
global count;
global theta_es;
global force;
global P;
global R;
global lambda;
temp=[u(1),u(3)];
if ( count < 20*paraNum)
    dataMatrix=[dataMatrix;temp];
    force=[force;u(5)];
    count=count+1;
    sys=0;
elseif (count == 20*paraNum)
    dataMatrix=[dataMatrix;temp];
    force=[force;u(5)];
    P=inv(dataMatrix'*dataMatrix);
    theta_es=P*dataMatrix'*force;
    count=count+1;
    disp('the initial estimation of theta is');
    disp(theta_es);
    
    sys=0;
else
    tempTheta=theta_es;
    
    R=P*temp'/(lambda+temp*P*temp');
    P=(eye(paraNum)-R*temp)*P/lambda;
    theta_es=theta_es+R*(u(5)-temp*theta_es)/lambda;
    sys=temp*tempTheta;
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
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate

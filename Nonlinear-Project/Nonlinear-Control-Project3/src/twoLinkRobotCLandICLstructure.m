function twoLinkRobotCLandICLstructure

% setup and initialization
       
% options for integration function
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);

% integrate (you can send the paramters theta to the dynamics as seen below)
[t,STATES] = ode45(@(t,X) twoLinkdynamics(t,X,theta),[0 tf],X0,opts);

% post computation for figures

% plot figures
end

function [XDot] = twoLinkdynamics(t,X,theta)
% initialize global variables for use in plot later.

% compute desired trajectory and needed derivatives (i.e., qd, qdDot, qdDotDot)
%missing%

% select gains (i.e., K, Kcl/Kicl, alpha, Gamma)
%missing%

% parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
%missing%

% compute current q and qDot for convenience
%missing%

% compute cos(x2) and sin(x2) for convenience
%missing%

% compute current matrices for the dynamics (i.e., M, Vm, ff)
%missing%

% compute current regression matrix (Y2 for gradient term/ non cl/icl)
%missing%

% set lambda for history stack min eigenvalue condition
lambda  = 0;

% determine if the history stack meets the min eigenvalue condition
if pos == 0 %if min eigenvalue of history stack less than lambda

    % compute qDotDot for Y1i
    dt        = t - timeprev; % determine delta time.
    dq        = qDot - qDotprev; % determine delta qDot.
    qDotDot   = dq/dt; % determine qDotDot.

    %if first loop (cannot determine dq or dt), initiallize qDotDot.
    if dt == 0
        qDotDot   = [0;0];
    end

    %determine Y1i (Y1 for this loop)
    %missing%

    % determine new history stack (summation)
    Y1hist    = Y1hist + Y1i'*Y1i;
    
    % optional: save the Y1i data in an array instead, then add/remove data
    % from history stack to maximize min eigenvalue of Y1hist.

    % determine min eigenvalue of history stack
    Y1eigenVal = eig(Y1hist);

    % check min eigenvalue condition
    Y1eigenValMin = min(Y1eigenVal);
    if (Y1eigenValMin > lambda)
       pos = 1;
    else
       pos = 0;
    end
end

% design controller (i.e., u)
%missing%

% determine which update law to use (i.e., thetaHatDot)
if pos == 1
    %(after eigenvalues condition met for Y1hist)
    
    % for CL:
    % compute new cl_sumY (function of: Yi,ui,thetaHati), 
    % assuming new data in history stack
    
    % for ICL:
    % compute new icl_sumY(script) (function of: Yi(script),ui,x(ti),x(ti-dt),thetaHati),
    % assuming new data in history stack

    % compute thetaHatDot
    %missing%
else
    % (before eigenvalues condition met for Y1hist)

    % compute thetaHatDot (no cl/icl term)
    %missing%
    
    % for CL:
    % compute cl_sumY (function of: Yi,ui, thetaHat) for use later
    %missing%

    % for ICL:
    % compute icl_sumY(script) (function of: Yi(script),ui,x(ti),x(ti-deltat),thetaHati) for use later
    %missing%
end 

% compute current closed-loop errors for integration(i.e., eDot, rDot)
%missing%

% update "previous" variables for next loop
%(i.e., qDotprev, timeprev, Y1iprev, Uiprev)
%missing%

% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot];
end


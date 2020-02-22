function CLmethod

% setup and initialization
hold on;
clear all


p1       = 3.473;
p2       = 0.196;
p3       = 0.242;
f1       = 5.3;
f2       = 1.1;

theta    = [p1;p2;p3;f1;f2];

tf   = 200;

% Initial condition vector (X0 must be same size and "form" as X and XDot below)
% (i.e., in this sim, X0 = [e0;r0;thetahat0])
X0   = [4;10;3;2;1;1;1;1;1];

global his;
keyset  = {'time','ui','Y1i','Eigen'};
keyvalue = {[],[],[],[]}; % Initializtion, will be replaced in
                                   % ODE iterations.
his = containers.Map(keyset,keyvalue);

% options for integration function
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);

% integrate (you can send the paramters theta to the dynamics as seen below)
[t,STATES] = ode45(@(t,X) CLdynamics(t,X,theta),[0 tf],X0,opts);

e  = STATES(:,1:2)';
thetaHat = STATES(:,5:9)';

%{
figure(1)
plot(t,sqrt(sum(e.*e,1)),'-','LineWidth',2)


figure(2)
plot(t,repmat(theta,1,length(t)),'-','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,thetaHat,':','LineWidth',2)
hold off

figure(3)
plot(t,thetaHat-repmat(theta,1,length(t)),'-','LineWidth',2)

figure(4)
plot(his('time'),his('ui'),'-','LineWidth',2)

figure(5)
plot(his('time'),his('Eigen'),'-','LineWidth',1)

figure(6)
%}
thetatilde = thetaHat-repmat(theta,1,length(t));
abstilde = thetatilde.*thetatilde;
abstilde = sum(abstilde,1);
plot(t,sqrt(abstilde),'-','LineWidth',2);

end

function [XDot] = CLdynamics(t,X,theta)

% initialize global variables

global his;

persistent flag;
if isempty(flag)
    flag = 0;
end

persistent Y1hist;
if isempty(Y1hist)
    Y1hist = 0;
end

persistent qDotprev;
if isempty(qDotprev)
    qDotprev = 0;
end

persistent timeprev;
if isempty(timeprev)
    timeprev = 0;
end

persistent Y1eigenValMin;
if isempty(Y1eigenValMin)
    Y1eigenValMin = 0;
end

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);

% compute desired trajectory and needed derivatives (i.e., qd, qdDot, qdDotDot)
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];

% select gains (i.e., K, Kcl/Kicl, alpha, Gamma)
a = 0.5;
gamma = [5 0 0 0 0; ...
         0 1 0 0 0; ...
         0 0 1 0 0; ...
         0 0 0 1 0; ...
         0 0 0 0 1];
K = 1;
Kcl = 10e-6;

% parse current states (X is the same size and "form" as X0)
e        = [X(1);X(2)];
r        = [X(3);X(4)];
thetaHat = [X(5);X(6);X(7);X(8);X(9)];

% compute current q and qDot for convenience
q        = qd - e;
qDot     = a*e -r + qdDot;


% compute cos(x2) and sin(x2) for convenience
c2       = cos(q(2));
s2       = sin(q(2));

% compute current matrices for the dynamics (i.e., M, Vm, ff)
M        = [p1 + 2*p3*c2 p2 + p3*c2;p2 + p3*c2 p2];
Vm       = [-p3*s2*qDot(2) -p3*s2*(qDot(1) + qDot(2));p3*s2*qDot(1) 0];
fd       = [f1 0;0 f2];

eDot        = r - a*e;

% compute current regression matrix (Y2 for gradient term/ non cl/icl)
Y2 = [qdDotDot(1)+a*eDot(1),qdDotDot(2)+a*eDot(2), ...
    2*c2*qdDotDot(1)+c2*qdDotDot(2)-s2*qDot(2)*(qdDot(1)+a*e(1))-s2*(qDot(1)+qDot(2))*(qdDot(2)+a*e(2))+2*c2*a*eDot(1)+c2*a*eDot(2), ...
    qDot(1),0; ...
    0, qdDotDot(1)+qdDotDot(2)+a*eDot(1)+a*eDot(2), ...
    c2*qdDotDot(1)+s2*qDot(1)*(qdDot(1)+a*e(1))+c2*a*eDot(1), ...
    0, qDot(2)];

% set lambda for history stack min eigenvalue condition
lambda  = 1;

% determine if the history stack meets the min eigenvalue condition
if flag == 0 %if min eigenvalue of history stack less than lambda
    
    % if first loop (cannot determine dq or dt), initiallize qDotDot.
    if t == 0
        qDotDot   = [0;0];
    else
        % compute qDotDot for Y1i
        dt        = t - timeprev; % determine delta time.
        dq        = qDot - qDotprev; % determine delta qDot.
        qDotDot   = dq/dt; % determine qDotDot.
    end
    
    %determine Y1i (Y1 for this loop)
    y11      = qDotDot(1); 
    y12      = qDotDot(2); 
    y13      = 2*c2*qDotDot(1)+c2*qDotDot(2)-s2*qDot(2)*qDot(1)-s2*(qDot(1)+qDot(2))*qDot(2);
    y14      = qDot(1); 
    y15      = 0; 
    y21      = 0;
    y22      = qDotDot(1)+qDotDot(2); 
    y23      = c2*qDotDot(1)+s2*qDot(1)*qDot(1); 
    y24      = 0; 
    y25      = qDot(2); 
    Y1i       = [y11 y12 y13 y14 y15;y21 y22 y23 y24 y25];
    his('Y1i') = cat(3,his('Y1i'),Y1i);
    
    % determine new history stack (summation)
    if Y1i ~= inf & Y1i~= -inf & Y1i~= -inf
        Y1hist = Y1hist + Y1i'*Y1i;
    end

    % determine min eigenvalue of history stack
    Y1eigenVal = eig(Y1hist);
    Y1eigenValMin = min(Y1eigenVal);
    
    % check min eigenvalue condition
    if (Y1eigenValMin > lambda)
       flag = 1;
    else
       flag = 0;
    end
end

his('Eigen') = cat(1,his('Eigen'),Y1eigenValMin);
% design controller (i.e., u)
u = Y2*thetaHat + e + K*r;
his('time') = cat(1,his('time'),t);
his('ui') = cat(2,his('ui'),u);

% determine which update law to use (i.e., thetaHatDot)
if flag == 1
    %(after eigenvalues condition met for Y1hist)
    
    % for CL:
    % compute new cl_sumY (function of: Yi,ui,thetaHati), 
    % assuming new data in history stack
    
    % for ICL:
    % compute new icl_sumY(script) (function of: Yi(script),ui,x(ti),x(ti-dt),thetaHati),
    % assuming new data in history stack

    % compute thetaHatDot
    iter = size(his('Y1i'));
    iter = iter(3);
    
    ui = his('ui');
    Y1i = his('Y1i');
    cl_sumY = 0;
    for i = 1:iter
        if Y1i(:,:,i)~= inf & Y1i(:,:,i)~= -inf & Y1i(:,:,i)~= nan
            cl_sumY = cl_sumY + Y1i(:,:,i)'*(ui(:,i)-Y1i(:,:,i)*thetaHat);
        end
    end
    thetaHatDot = gamma*Y2'*r + Kcl*gamma*cl_sumY;
else
    % (before eigenvalues condition met for Y1hist)

    % compute thetaHatDot (no cl/icl term)
    
    % for CL:
    % compute cl_sumY (function of: Yi,ui, thetaHat) for use later
    thetaHatDot = gamma*Y2'*r;

    % for ICL:
    % compute icl_sumY(script) (function of: Yi(script),ui,x(ti),x(ti-deltat),thetaHati) for use later
    %missing%
end 

% compute current closed-loop errors for integration(i.e., eDot, rDot)
rDot        = a*eDot - M\(-Vm*qDot-fd*qDot+u) + qdDotDot;

% update "previous" variables for next loop
%(i.e., qDotprev, timeprev, Y1iprev, Uiprev)
qDotprev = qDot;
timeprev = t;


% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot];
end

function RLC
close all
%Set up parameters for sim
p1       = 3.473;
p2       = 0.196;
p3       = 0.242;
f1       = 5.3;
f2       = 1.1;
fs1      = 1.2;
fs2      = 0.4;

% Stacked parameter vector
theta    = [p1;p2;p3;f1;f2;fs1;fs2];

% Simulation final time
tf   = 50;

% Initial condition vector (X0 must be same size and "form" as X and XDot below)
% (i.e., in this sim, X0 = [e0;r0;thetahat0])
X0   = [4;10;3;2;5;5];

% Options for integration function
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);

global his;
keyset  = {'time','tau'};
keyvalue = {10,[10;10]};
his = containers.Map(keyset,keyvalue);


% Integrate (you can send the paramters theta to the dynamics as seen below)
[t,STATES] = ode45(@(t,X) RLCdynamics(t,X,theta),[0 tf],X0,opts);
% Set up desired trajectory data for plots (enter desired trajectory for your simulation)
qd = [cos(0.5*t) 2*cos(t)]';

% Parse integrated states (STATES is the same "form" as X0)
% (i.e., in this sim, STATES = [e r thetahat] over all time);
e  = STATES(:,1:2)';
r  = STATES(:,3:4)';
thetaHat = STATES(:,5:6)';

% Compute x from e and xd for plotting purposes
q  = e + qd;
% Plot the actual vs desired trajectories
figure(1)
plot(t,qd,'-','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,q,':','LineWidth',2)
hold off

% Plot the filtered tracking error
figure(2)
plot(t,e,'LineWidth',2)

% Plot the adaptive estimates vs actual parameters
figure(3)
plot(t,repmat(theta(6:7),1,length(t)),'-','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,thetaHat,':','LineWidth',2)
hold off

figure(4)
plot(t,thetaHat-repmat(theta(6:7),1,length(t)),'-','LineWidth',2)


thetaTilda = thetaHat-repmat(theta(6:7),1,length(t));
thetaTilda = thetaTilda.*thetaTilda;
thetaTilda = sqrt(sum(thetaTilda));
figure(5)
plot(t,thetaTilda,'LineWidth',2)
legend({'$\left \| \tilde {\theta} \right \|$'},'Interpreter','latex')


size(his('time'))
size(his('tau'))
size(t)

time = his('time');
tau = his('tau');
figure(6)
plot(time,tau,'-','LineWidth',2)

figure(7)
abs_e = sqrt(sum(e.*e));
plot(t,abs_e,'LineWidth',2)

figure(8)
plot(time,sqrt(sum(tau.*tau)),'LineWidth',2)



function [XDot] = RLCdynamics(t,X,theta)

global his;

persistent WhatHis;
if isempty(WhatHis)
    WhatHis = containers.Map('KeyType','double','ValueType','any');
end

T = 4*pi;

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);
fs1 = theta(6);
fs2 = theta(7);

% Select gains for controller
K        = 5; 
Kn       = 2;
KL       = 1;
a        = 1; 


% Desired trajectory and needed derivatives
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];

% Parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
e        = [X(1);X(2)];
r        = [X(3);X(4)];
thetaHat = [X(5);X(6)];

% Compute current x and xDot for convenience
q        = qd - e;
qDot     =  - r + a*e + qdDot;

% Compute cos(x2) and sin(x2) for convenience
c2       = cos(q(2));
s2       = sin(q(2));

% Compute current matrices for the dynamics
M        = [p1 + 2*p3*c2 p2 + p3*c2;p2 + p3*c2 p2];
Vm       = [-p3*s2*qDot(2) -p3*s2*(qDot(1) + qDot(2));p3*s2*qDot(1) 0];
fd       = [f1 0;0 f2];
fs       = [fs1 0;0 fs2];


% Compute current regression matrix
%{
cd2       = cos(qd(2));
sd2       = sin(qd(2));
yd11      = qdDotDot(1); 
yd12      = qdDotDot(2); 
yd13      = 2*cd2*qdDotDot(1)+cd2*qdDotDot(2)-sd2*qdDot(2)*qdDot(1)-sd2*(qdDot(1)+qdDot(2))*qdDot(2);
yd14      = qdDot(1); 
yd15      = 0; 
yd16      = sign(qdDot(1));
yd17      = 0;

yd21      = 0;
yd22      = qdDotDot(1)+qdDotDot(2); 
yd23      = cd2*qdDotDot(1)+sd2*qdDot(1)*qdDot(1); 
yd24      = 0; 
yd25      = qdDot(2); 
yd26      = 0;
yd27      = sign(qdDot(2));
Yd       = [yd11 yd12 yd13 yd14 yd15 yd16 yd17;yd21 yd22 yd23 yd24 yd25 yd26 yd27];
%}

ys11 = sign(qDot(1));
ys12 = 0;
ys21 = 0;
ys22 = sign(qDot(2));
Ys   = [ys11, ys12;ys21, ys22];

% Design u
Beta = 10;

if t<=T
    What = [0;0];
    WhatHis(t) = What;
else
    temp = t -T;
    temp_2 = WhatHis.keys;
    temp_3 = cell2mat(temp_2);
    if length(temp_3)>5000
        remove(WhatHis,temp_3(1));
    end
    [o,index] = min(abs(temp_3-temp));
    index = temp_3(index);
    sat = Sat(WhatHis(index),Beta);
    What = sat + KL*r;
    WhatHis(t) = What;
end


p = 0.1;
u = K*r + Ys*thetaHat + e + Kn*(p*p)*r + What;

prevtimes = his('time');
prevtimes = prevtimes(t > prevtimes);
prevtaus = his('tau');
prevtaus = prevtaus(:,t > prevtimes);
his('time') = [prevtimes, t];
his('tau') = [prevtaus, u];

% Compute current closed-loop dynamics
friction_m = [sign(qDot(1));sign(qDot(2))];
eDot        = r - a*e;
rDot        = a*eDot + qdDotDot - M\(-Vm*qDot-fd*qDot-fs*friction_m+u); %Enter the expression
gamma = [1, 0; ...
         0, 1];
     
thetaHatDot = gamma*Ys.'*r; %Enter the expression

t
% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot];
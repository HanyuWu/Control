function traditional
close all
%Set up parameters for sim
p1       = 3.473;
p2       = 0.196;
p3       = 0.242;
f1       = 5.3;
f2       = 1.1;

% Stacked parameter vector
theta    = [p1;p2;p3;f1;f2];

% Simulation final time
tf   = 1000;

% Initial condition vector (X0 must be same size and "form" as X and XDot below)
% (i.e., in this sim, X0 = [e0;r0;thetahat0])
X0   = [4;10;3;2;1;1;1;1;1];

% Options for integration function
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);
% Integrate (you can send the paramters theta to the dynamics as seen below)
[t,STATES] = ode45(@(t,X) tradynamics(t,X,theta),[0 tf],X0,opts);
% Set up desired trajectory data for plots (enter desired trajectory for your simulation)
qd = [cos(0.5*t) 2*cos(t)]';

% Parse integrated states (STATES is the same "form" as X0)
% (i.e., in this sim, STATES = [e r thetahat] over all time);
e  = STATES(:,1:2)';
r  = STATES(:,3:4)';
thetaHat = STATES(:,5:9)';

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
plot(t,r,'--','LineWidth',2)

% Plot the adaptive estimates vs actual parameters
figure(3)
plot(t,repmat(theta,1,length(t)),'-','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,thetaHat,':','LineWidth',2)
hold off

figure(4)
plot(t,thetaHat-repmat(theta,1,length(t)),'-','LineWidth',2)

index = 0;
tau = [0;0];
for i = t
    global tau;
    index = index+1;
    qd       = [cos(0.5*i);2*cos(i)];
    qdDot    = [-0.5*sin(0.5*i); -2*sin(i)];   %Enter the expression
    qdDotDot = [-0.25*cos(0.5*i); -2*cos(i)];  %Enter the expression 
    cd2       = cos(qd(2));
    sd2       = sin(qd(2));
    yd11      = qdDotDot(1); %Enter the expression
    yd12      = qdDotDot(2); %Enter the expression
    yd13      = 2*cd2*qdDotDot(1)+cd2*qdDotDot(2)-sd2*qdDot(2)*qdDot(1)-sd2*(qdDot(1)+qdDot(2))*qdDot(2); %Enter the expression
    yd14      = qdDot(1); %Enter the expression
    yd15      = 0; %Enter the expression
    yd21      = 0; %Enter the expression
    yd22      = qdDotDot(1)+qdDotDot(2); %Enter the expression
    yd23      = cd2*qdDotDot(1)+sd2*qdDot(1)*qdDot(1); %Enter the expression
    yd24      = 0; %Enter the expression
    yd25      = qdDot(2); %Enter the expression
    Yd       = [yd11 yd12 yd13 yd14 yd15;yd21 yd22 yd23 yd24 yd25];
    K        = 5;
    u        = -K*r(:,index) - e(:,index) + Yd*thetaHat(:,index);
    tau = horzcat(tau,u);

end
tausize = size(tau);
length_ = 1:tausize(2);
figure(5)
plot(length_,tau,'--','LineWidth',2)

function [XDot] = tradynamics(t,X,theta)

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);

% Select gains for controller
K        = 5; %Enter a number
a        = 1.5; %Enter a number

% Desired trajectory and needed derivatives
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];   %Enter the expression
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];  %Enter the expression 

% Parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
e        = [X(1);X(2)];
r        = [X(3);X(4)];
thetaHat = [X(5);X(6);X(7);X(8);X(9)];

% Compute current x and xDot for convenience
q        = e + qd;
qDot     = r - a*e + qdDot;

% Compute cos(x2) and sin(x2) for convenience
c2       = cos(q(2));
s2       = sin(q(2));

% Compute current matrices for the dynamics
M        = [p1 + 2*p3*c2 p2 + p3*c2;p2 + p3*c2 p2];
Vm       = [-p3*s2*qDot(2) -p3*s2*(qDot(1) + qDot(2));p3*s2*qDot(1) 0];
fd       = [f1 0;0 f2];

% Compute current regression matrix
cd2       = cos(qd(2));
sd2       = sin(qd(2));
yd11      = qdDotDot(1); %Enter the expression
yd12      = qdDotDot(2); %Enter the expression
yd13      = 2*cd2*qdDotDot(1)+cd2*qdDotDot(2)-sd2*qdDot(2)*qdDot(1)-sd2*(qdDot(1)+qdDot(2))*qdDot(2); %Enter the expression
yd14      = qdDot(1); %Enter the expression
yd15      = 0; %Enter the expression
yd21      = 0; %Enter the expression
yd22      = qdDotDot(1)+qdDotDot(2); %Enter the expression
yd23      = cd2*qdDotDot(1)+sd2*qdDot(1)*qdDot(1); %Enter the expression
yd24      = 0; %Enter the expression
yd25      = qdDot(2); %Enter the expression
Yd       = [yd11 yd12 yd13 yd14 yd15;yd21 yd22 yd23 yd24 yd25];

% u        = -K*r + M*qdDotDot + Vm*qDot + fd*qDot; %Enter the expression
% u        = -K*r - e
u        = -K*r + Yd*thetaHat;
% Compute current closed-loop dynamics
eDot        = r - a*e;
rDot        = a*eDot + M\(-Vm*qDot-fd*qDot+u) - qdDotDot; %Enter the expression
gamma = eye([5,5]);
thetaHatDot = -gamma*Yd.'*r; %Enter the expression

% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot];

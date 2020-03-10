function traditional_Rise
close all
% Set up dynamics for sim
p1       = 3.473;
p2       = 0.196;
p3       = 0.242;
f1       = 5.3;
f2       = 1.1;




% Simulation final time
tf   = 30;

% Initial condition vector (X0 must be same size and "form" as X and XDot below)
% (i.e., in this sim, X0 = [e1_0;e2_0;r_0;thetahat_0])

% define e2_0
e2_0 = [3;2];

% define intergrate part
intergrate_0 = [0;0];

theta    = [p1;p2;p3;f1;f2];

X0   = [4;10;e2_0;3;2;intergrate_0];


% Options for integration function
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);

% Integrate (you can send the paramters theta to the dynamics as seen below)
uout = [];
[t,STATES] = ode45(@(t,X) tra_rise(t,X,theta),[0 tf],X0,opts);

% Set up desired trajectory data for plots (enter desired trajectory for your simulation)
qd = [cos(0.5*t) 2*cos(t)]';

% Parse integrated states (STATES is the same "form" as X0)
% (i.e., in this sim, STATES = [e r thetahat] over all time);
e1  = STATES(:,1:2)';
e2  = STATES(:,3:4)';
r  = STATES(:,5:6)';
intergrate = STATES(:,7:8)';



% Compute x from e and xd for plotting purposes
q  = e1 + qd;


% Plot the actual vs desired trajectories
figure(1)
plot(t,qd,'-','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,q,':','LineWidth',2)
hold off

figure(2)
plot(t,e1,'b','LineWidth',2)


figure(3)
plot(t,e2,'-','LineWidth',2)

figure(4)
K        = 10;
uout = (K+1)*e2 - (K+1)*e2_0 + intergrate;
plot(t,uout,'-','LineWidth',2)


function [XDot] = tra_rise(t,X,theta)

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);

e2_0 = [3;2];

% Select gains for controller
K        = 10;
a1       = 1.5;
a2       = 1.5;
Beta     = 10; 


% Desired trajectory and needed derivatives
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];   %Enter the expression
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];  %Enter the expression 


% Parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
e1       = [X(1);X(2)];
e2       = [X(3);X(4)];
r        = [X(5);X(6)];
intergrate     = [X(7);X(8)];


% Compute current x and xDot for convenience
q        = -e1 + qd;
qDot     = -e2 + a1*e1 + qdDot;

% Compute cos(x2) and sin(x2) for convenience
c2       = cos(q(2));
s2       = sin(q(2));

% Compute current matrices for the dynamics 
% with interuption interruption Taud
M        = [p1 + 2*p3*c2 p2 + p3*c2;p2 + p3*c2 p2];
Vm       = [-p3*s2*qDot(2) -p3*s2*(qDot(1) + qDot(2));p3*s2*qDot(1) 0];
fd       = [f1 0;0 f2];

Taud     = [0.5*cos(0.5*t); sin(t)];


% Design RISE term
intergrate_dot = (K+1)*a2*e2 + Beta*sign(e2);
miu1 = (K+1)*e2 - (K+1)*e2_0 + intergrate;

% Design input u
u = miu1;

e1Dot = e2 - a1*e1;
e2Dot = a1*e1Dot - M\(-Vm * qDot -fd*qDot + u - Taud) + qdDotDot;

XDot   = [e1Dot;e2Dot;0;0;intergrate_dot];

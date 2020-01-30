function compare
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
tf   = 500;

% Initial condition vector (X0 must be same size and "form" as X and XDot below)
% (i.e., in this sim, X0 = [e0;r0;thetahat0])
X0   = [4;10;3;2;1;1;1;1;1];

% define uf and Ydf initial value
uf   = [2;2];
Ydf =  [1;1;1;1;1;1;1;1;1;1];


X1   = [X0;uf;Ydf];

P0 = eye([5,5]);
P0 = reshape(P0,[25,1]);
X2   = [X1;P0];

% Options for integration function
opts = odeset('RelTol',1e-3,'AbsTol',1e-3);

% Integrate (you can send the paramters theta to the dynamics as seen below)
[t,STATES0] = ode45(@(t,X) tradynamics(t,X,theta),[0 tf],X0,opts);
thetaHat0 = STATES0(:,5:9)';
figure()
plot(t,thetaHat0,'LineWidth',2)
hold on

[t,STATES1] = ode45(@(t,X) compositedynamics2(t,X,theta),[0 tf],X1,opts);
thetaHat1 = STATES1(:,5:9)';
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,thetaHat1,':','LineWidth',2)
hold on

ax = gca;
ax.ColorOrderIndex = 1;
plot(t,repmat(theta,1,length(t)),'-','LineWidth',2)
hold on

[t,STATES2] = ode45(@(t,X) leastsquaredynamics2(t,X,theta),[0 tf],X2,opts);
thetaHat2 = STATES2(:,5:9)';
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,thetaHat2,'-.','LineWidth',2)



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
u        = -K*r - e + Yd*thetaHat;


% Compute current closed-loop dynamics
eDot        = r - a*e;
rDot        = a*eDot + M\(-Vm*qDot-fd*qDot+u) - qdDotDot; %Enter the expression
gamma = eye([5,5]);
thetaHatDot = -gamma*Yd.'*r; %Enter the expression

% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot];

function [XDot] = compositedynamics2(t,X,theta)

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);

% Select gains for controller
K        = 5; % Enter a number
a        = 1.5; % Enter a number
Beta     = 1;  % Torque filter gain, define it by yourself
gamma = eye([5,5]);   


% Desired trajectory and needed derivatives
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];   %Enter the expression
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];  %Enter the expression 

% Parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
e        = [X(1);X(2)];
r        = [X(3);X(4)];
thetaHat = [X(5);X(6);X(7);X(8);X(9)];
uf       = [X(10);X(11)];
Ydf      = [X(12),X(13),X(14),X(15),X(16);X(17),X(18),X(19),X(20),X(21)];



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

% Design controller

% u        = -K*r + M*qdDotDot + Vm*qDot + fd*qDot; %Enter the expression
% u        = -K*r - e
u        = -K*r + Yd*thetaHat;
eDot        = r - a*e;
rDot        = a*eDot + M\(-Vm*qDot-fd*qDot+u) - qdDotDot; %Enter the expression

miu = uf - Ydf*thetaHat;
thetaHatDot = -gamma*Yd.'*r + gamma*Ydf.'*miu; %Enter the expression
uf_dot      =  Beta*u - Beta*uf;
Ydf_dot = Beta*Yd -Beta*Ydf;
Ydf_dot = reshape(Ydf_dot',[10,1]);
% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot;uf_dot;Ydf_dot];

function [XDot] = leastsquaredynamics2(t,X,theta)

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);

% Select gains for controller
K        = 5; % Enter a number
a        = 1.5; % Enter a number
Beta     = 1;  % Torque filter gain, define it by yourself


% Desired trajectory and needed derivatives
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];   %Enter the expression
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];  %Enter the expression 

% Parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
e        = [X(1);X(2)];
r        = [X(3);X(4)];
thetaHat = [X(5);X(6);X(7);X(8);X(9)];
uf       = [X(10);X(11)];
Ydf      = [X(12),X(13),X(14),X(15),X(16);X(17),X(18),X(19),X(20),X(21)];
P        = X(22:end);
P        = reshape(P,[5,5]);

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

% Design controller

% u        = -K*r + M*qdDotDot + Vm*qDot + fd*qDot; %Enter the expression
% u        = -K*r - e
u           = -K*r + Yd*thetaHat;
eDot        = r - a*e;
rDot        = a*eDot + M\(-Vm*qDot-fd*qDot+u) - qdDotDot; %Enter the expression

miu = uf - Ydf*thetaHat;
thetaHatDot = -P*Yd.'*r + P*Ydf.'*miu; %Enter the expression
uf_dot      =  Beta*u - Beta*uf;

Ydf_dot = Beta*Yd -Beta*Ydf;
Ydf_dot = reshape(Ydf_dot',[10,1]);

Pdot = -P*(Ydf.')*Ydf*P;
Pdot = reshape(Pdot,[25,1]);
% Stacked dynamics vector (XDot is the same size and "form" as X)
XDot        = [eDot;rDot;thetaHatDot;uf_dot;Ydf_dot;Pdot];


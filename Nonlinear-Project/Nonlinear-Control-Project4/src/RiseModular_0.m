function RiseModular_0
%Set up dynamics for sim
hold on;

p1       = 3.473;
p2       = 0.196;
p3       = 0.242;
f1       = 5.3;
f2       = 1.1;

theta    = [p1;p2;p3;f1;f2];

% Stacked parameter vector

% Simulation final time
tf   = 30;

% Initial condition vector (X0 must be same size and "form" as X and XDot below)
% (i.e., in this sim, X0 = [e1_0;e2_0;r_0;thetahat_0])

% define e2_0
e2_0     = [3;2];

X0   = [4;10;e2_0;3;2;1;1;1;1;1];



% define uf initial value
uf   = [2;2];

% define ydf initial value
Ydf =  [1;1;1;1;1;1;1;1;1;1];

% define the intergration part in RISE 
Rise_intergrate_1 = [0;0];
Rise_intergrate_2 = [0;0];

% X0   = [X0;uf;Ydf;Rise_intergrate_1;Rise_intergrate_2];
X0   = [X0;Rise_intergrate_1]


% Options for integration function
opts = odeset('RelTol',exp(-3),'AbsTol',exp(-3));

% Integrate (you can send the paramters theta to the dynamics as seen below)
[t,STATES] = ode45(@(t,X) composite_rise1(t,X,theta),[0 tf],X0,opts);

% Set up desired trajectory data for plots (enter desired trajectory for your simulation)
qd = [cos(0.5*t) 2*cos(t)]';

% Parse integrated states (STATES is the same "form" as X0)
% (i.e., in this sim, STATES = [e r thetahat] over all time);
e1  = STATES(:,1:2)';
e2  = STATES(:,3:4)';
r  = STATES(:,5:6)';
thetaHat = STATES(:,7:11)';
Rise_intergrate_1  = STATES(:,12:13)';


% Compute x from e and xd for plotting purposes
q  = -e1 + qd;

% Plot the actual vs desired trajectories
figure(1)
plot(t,qd,'-','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,q,':','LineWidth',2)
hold off

figure(2)
plot(t,e1,'k-','LineWidth',2)


figure(3)
plot(t,repmat(theta,1,length(t)),':','LineWidth',2)
hold on
ax = gca;
ax.ColorOrderIndex = 1;
plot(t,thetaHat,'-','LineWidth',2)
hold off

index = 0;
global tau;
tau = [];
for i = t'
    index = index+1;
    qd       = [cos(0.5*i);2*cos(i)];
    qdDot    = [-0.5*sin(0.5*i); -2*sin(i)];
    qdDotDot = [-0.25*cos(0.5*i); -2*cos(i)];  
    cd2       = cos(qd(2));
    sd2       = sin(qd(2));
    yd11      = qdDotDot(1); 
    yd12      = qdDotDot(2); 
    yd13      = 2*cd2*qdDotDot(1)+cd2*qdDotDot(2)-sd2*qdDot(2)*qdDot(1)-sd2*(qdDot(1)+qdDot(2))*qdDot(2); %Enter the expression
    yd14      = qdDot(1); 
    yd15      = 0; 
    yd21      = 0; 
    yd22      = qdDotDot(1)+qdDotDot(2); 
    yd23      = cd2*qdDotDot(1)+sd2*qdDot(1)*qdDot(1); 
    yd24      = 0; 
    yd25      = qdDot(2); 
    Yd       = [yd11 yd12 yd13 yd14 yd15;yd21 yd22 yd23 yd24 yd25];
    K        = 5;
    u        = Yd*thetaHat(:,index) + (K+1)*e2(:,index) - (K+1)*e2_0 + Rise_intergrate_1(:,index);
    tau = horzcat(tau,u);
end

tausize = size(tau)
length_ = 1:tausize(2);
figure(4)
plot(length_,tau,'-','LineWidth',2)


figure(5)
plot(t,thetaHat-repmat(theta,1,length(t)),'-','LineWidth',2)
%}


function [XDot] = composite_rise1(t,X,theta)

% Parse parameter vector
p1 = theta(1);
p2 = theta(2);
p3 = theta(3);
f1 = theta(4);
f2 = theta(5);

e2_0     = [3;2];

% Select gains for controller
Ks        = 10; 
% K2       = 25;
a1       = 2; 
a2       = 3;
Beta     = 1; 
%{
gamma = [10 0 0 0 0; ...
    0 1 0 0 0; ...
    0 0 1 0 0; ...
    0 0 0 10 0; ...
    0 0 0 0 1];   
%}

% Desired trajectory and needed derivatives
qd       = [cos(0.5*t);2*cos(t)];
qdDot    = [-0.5*sin(0.5*t); -2*sin(t)];   %Enter the expression
qdDotDot = [-0.25*cos(0.5*t); -2*cos(t)];  %Enter the expression 
qdDotDotDot = [0.125*sin(0.5*t); 2*sin(t)];


% Parse current states (X is the same size and "form" as X0)
% (i.e., in this sim, X = [e;r;thetahat])
e1       = [X(1);X(2)];
e2       = [X(3);X(4)];
r        = [X(5);X(6)];
thetaHat = [X(7);X(8);X(9);X(10);X(11)];
% uf       = [X(12);X(13)];
% Ydf      = [X(14),X(15),X(16),X(17),X(18);X(19),X(20),X(21),X(22),X(23)];
Rise_intergrate_1 = [X(12);X(13)];
% Rise_intergrate_2 = [X(26);X(27)];


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

Taud     = [0; 0];

% Compute current regression matrix
cd2       = cos(qd(2));
sd2       = sin(qd(2));

yd11      = qdDotDot(1);
yd12      = qdDotDot(2); 
yd13      = 2*cd2*qdDotDot(1)+cd2*qdDotDot(2)-sd2*qdDot(2)*qdDot(1)-sd2*(qdDot(1)+qdDot(2))*qdDot(2); %Enter the expression
yd14      = qdDot(1); 
yd15      = 0; 
yd21      = 0; 
yd22      = qdDotDot(1)+qdDotDot(2);
yd23      = cd2*qdDotDot(1)+sd2*qdDot(1)*qdDot(1); 
yd24      = 0; 
yd25      = qdDot(2); 
Yd       = [yd11 yd12 yd13 yd14 yd15;yd21 yd22 yd23 yd24 yd25];

%{
% Compute differential of regression matrix
ydDot11      = qdDotDotDot(1); 
ydDot12      = qdDotDotDot(2); 
ydDot13      = -2*sd2*qdDot(2)*qdDotDot(1)+2*cd2*qdDotDotDot(1) ...
    -sd2*qdDot(2)*qdDotDot(2) + cd2*qdDotDotDot(2) ...
    -cd2*qdDot(2)*qdDot(2)*qdDot(1) ...
    -sd2*qdDotDot(2)*qdDot(1)-sd2*qdDot(2)*qdDotDot(1) ...
    -cd2*qdDot(2)*(qdDot(1)+qdDot(2))*qdDot(2) ...
    -sd2*(qdDotDot(1)+qdDotDot(2))*qdDot(2) ...
    -sd2*(qdDot(1)+qdDot(2))*qdDotDot(2);
ydDot14      = qdDotDot(1);
ydDot15      = 0; 
ydDot21      = 0; 
ydDot22      = qdDotDotDot(1)+qdDotDotDot(2); 
ydDot23      = -sd2*qdDot(2)*qdDotDot(1) ...
    + cd2*qdDotDotDot(1) ...
    + cd2*qdDot(2)*qdDot(1)*qdDot(1) ...
    + 2*sd2*qdDotDot(1)*qdDot(1);
ydDot24      = 0; 
ydDot25      = qdDotDot(2); 
YdDot       = [ydDot11 ydDot12 ydDot13 ydDot14 ydDot15;ydDot21 ydDot22 ydDot23 ydDot24 ydDot25];
%}

% Design RISE term
Rise_intergrate_1_Dot = (Ks+1)*a2*e2 + Beta*sign(e2);
miu1 = (Ks+1)*e2 - (Ks+1)*e2_0 + Rise_intergrate_1;

% Design input u
u = Yd*thetaHat + miu1;
e1Dot = e2 - a1*e1;
e2Dot = a1*e1Dot - M\(-Vm * qDot -fd*qDot + u - Taud) + qdDotDot;

% In this problem, we don't use rDot, but we suppose we can get 
% thetahatdot, and intergration of r can be approached with e2,
% and e2 is known. So in this case, we just use r for simplicty.
rDot = [0;0];  
r = e2Dot + a2*e2;

%% Design thetaHatDot

%{
Ydf_dot = Beta*Yd -Beta*Ydf;
uf_dot = Beta*u - Beta*uf;

ufhat = Ydf*thetaHat + Rise_intergrate_2;
E = uf - ufhat;
thetaHatDot = gamma*YdDot'*r + gamma*Ydf_dot'*E;
Rise_intergrate_2_Dot = K2*E + Beta*sign(E);
%}

thetaHatDot = [0;0;0;0;0];

t

%%
% Ydf_dot = reshape(Ydf_dot',[10,1]);
XDot        = [e1Dot;e2Dot;rDot;thetaHatDot; ...
    Rise_intergrate_1_Dot];

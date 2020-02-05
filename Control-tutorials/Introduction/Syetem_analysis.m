% For a BIBO system

s = tf('s');
G = 1/(s^2+2*s+5);
pole(G);

%% Convert to SS representation

[A,B,C,D] = ssdata(G)
eig(A)
% The stability of a system may also be found from the state-space 
% representation. In fact, the poles of the transfer function are the 
% eigenvalues of the system matrix A.

%% Step response  (First order system)

k_dc = 5;   % dc gain
Tc = 10;    % Time constant
u = 2;      % step input

s = tf('s');
G = k_dc/(Tc*s + 1);  
%  http://ctms.engin.umich.edu/CTMS/index.php?example=Introduction&section=SystemAnalysis#1

step(u*G);
linearSystemAnalyzer('step',G)

%% Bode plots

bode(G)
linearSystemAnalyzer('bode',G)

%% 


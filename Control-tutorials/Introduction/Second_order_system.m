% Second-Order Systems  16520020710111200020
%% Underdamped Systems

k_dc = 1;     % DC gain

w_n = 10;     % Natural Frequency (The natural frequency is the 
              % frequency (in rad/s) that the system will oscillate at when
              % there is no damping, )
              
zeta = 0.2;   % Damping Ratio
s = tf("s");
G1 = k_dc * w_n^2/ (s^2 + 2*zeta*w_n*s + w_n^2);

pzmap(G1);    % pole-zero diagram
axis([-3 1 -15 15])

step(G1)
axis([0 3 0 2])

%% Overdamped System

zeta = 2.0;

G2 = k_dc*w_n^2/(s^2 + 2*zeta*w_n*s + w_n^2);

pzmap(G2)
axis([-20 1 -1 1])

step(G2)
axis([0 1.5 0 1.5])

%% Critically-Damped Systems

zeta = 1;

G3 = k_dc*w_n^2/(s^2 + 2*zeta*w_n*s + w_n^2);

pzmap(G3)
axis([-11 1 -1 1])

step(G3)
axis([0 1.5 0 1.5])

%% Undamped System

zeta = 0;

G4 = k_dc*w_n^2/(s^2 + 2*zeta*w_n*s + w_n^2);

pzmap(G4)
axis([-1 1 -15 15])

step(G4)
axis([0 5 -0.5 2.5])

%% Bode Plot (overall)

bode(G1,G2,G3,G4)
legend('underdamped: zeta < 1','overdamped: zeta > 1','critically-damped: zeta = 1','undamped: zeta = 0')

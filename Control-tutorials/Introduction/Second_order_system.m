% Second-Order Systems 16520020710111200020
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


s = tf('s');
G = 50/(s^3 + 9*s^2 + 30*s +40);

bode(G)
grid on
title('Bode Plot with No Gain')

% Add a gain = 100;

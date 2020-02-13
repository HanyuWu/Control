%% PID overview
Kp = 1;
Ki = 1;
Kd = 1;

s = tf('s');
C = Kp + Ki/s + Kd*s; % This one is not a proper models



%%
%{
Suppose we have a simle mass-spring-damper system, 
and the governing equation of this system is:
m* \ddot{x} + b* \dot{x} + k*x = F

Parameters setting

m = 1 kg
b = 10 N s/m
k = 20 N/m
F = 1 N

X/F = 1/(ms^2 + bs + k)
%}

s = tf('s');
P = 1/(s^2+10*s+20)
step(P)   % open-loop step response.

%%
% Add Proportional Control
Kp = 300;
C = pid(Kp);
T = feedback(P*C,1);    % Unity-feedback System
t = 0:0.01:2;
step(T,t);

%%
% Add Proportional-Derivative Control
% hold on;
Kp = 300;
Kd = 10;

C = pid(Kp, 0 , Kd);
T = feedback(C*P,1);

t = 0:0.01:2;
step(T,t)

%%
% Add Proportional-Integral Control
% hold on;
Kp = 30;
Ki = 70;

C = pid(Kp,Ki,0);
T = feedback(C*P,1);

t = 0:0.01:2;
step(T,t)

%% 
% Add PID Control
Kp = 350;
Ki = 300;
Kd = 50;
C = pid(Kp,Ki,Kd);
T = feedback(C*P,1);

t = 0:0.01:2;
step(T,t)

%% Automatic PID tuning

pidTuner(P,'p')




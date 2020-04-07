clear
clc

%% Parameters

%% GAINS

k = 50;
kn = 10;
ks = 1;
alpha = 10;
gamma1 = 5;
gamma2 = 10;

v_initial = zeros(7,5);
w_initial = zeros(6,2);


%% Run Simulation
[t,~,states,u,qd,f_hat,f,w_hat,v_hat] = sim('Model_Composite_2Link_NN');

%% Analysis/Plot

states = [states(:,3:4),states(:,1:2)]; % reorder states to [pos1, pos2, vel1, vel2]
error = qd-states;
rms = sqrt(mean(error.^2))*180/pi;

figure(2)
plot(t,u(:,1))
title('Link 1 Control')
xlabel('Time (s)')
ylabel('Control Torque (N-m)')
axis([0,25,-250,250])
saveas(figure(2),'Link 1 Control sm.png')

figure(3)
plot(t,u(:,2))
title('Link 2 Control')
xlabel('Time (s)')
ylabel('Control Torque (N-m)')
axis([0,25,-30,30])
saveas(figure(3),'Link 2 Control sm.png')
 
figure(4)
plot(t,error(:,1)*180/pi,t,error(:,3)*180/pi)
title('Link 1 Position Errors')
legend('q1 Error','q1Dot Error')
xlabel('Time (s)')
ylabel('Link Position and Velocity Error (deg, deg/s)')
axis([0,25,-50,50])
saveas(figure(4),'Link 1 Position Errors sm.png')

figure(5)
plot(t,error(:,2)*180/pi,t,error(:,4)*180/pi)
title('Link 2 Position Errors')
legend('q2 Error','q2Dot Error')
xlabel('Time (s)')
ylabel('Link Position and Velocity Error (deg, deg/s)')
axis([0,25,-50,50])
saveas(figure(5),'Link 2 Position Errors sm.png')
 
figure(6)
plot(t,(f(:,1)-f_hat(:,1)))
title('Link 1 Parameter Estimate Errors')
xlabel('Time (s)')
ylabel('Percent Error (%)')
axis([0,25,-50,50])
saveas(figure(6),'Link 1 Parameter Estimate Errors sm.png')
  
figure(7)
plot(t,(f(:,2)-f_hat(:,2)))
title('Link 2 Parameter Estimate Errors')
xlabel('Time (s)')
ylabel('Percent Error (%)')
axis([0,25,-50,50])
saveas(figure(7),'Link 2 Parameter Estimate Errors sm.png')

figure(8)
plot(t,sqrt(sum(error.*error,2)))
title('||e||')
xlabel('Time (s)')
ylabel('||e||')
axis([0,25,-50,50])

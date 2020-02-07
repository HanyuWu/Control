t0 = 0;
tfinal = 10;
h = 0.1;
x0 = 10;
Fuc = @(t,x) -0.5*x^2;
xout = ode1(Fuc, t0, h, tfinal, x0);
plot((t0:h:tfinal)',xout);
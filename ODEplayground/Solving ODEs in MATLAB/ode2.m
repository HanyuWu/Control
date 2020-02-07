   function yout = ode2(F,t0,h,tfinal,y0)
   % ODE2  A simple ODE solver.
   %   yout = ODE2(F,t0,h,tfinal,y0) uses a midpoint
   %   rule with fixed step size h on the interval
   %      t0 <= t <= tfinal
   %   to solve
   %      dy/dt = F(t,y)
   %   with y(t0) = y0.

   %   Copyright 2014 - 2015 The MathWorks, Inc.

   
      y = y0;
      yout = y;
      for t = t0 : h : tfinal-h
         s1 = F(t,y);
         s2 = F(t+h/2, y+h*s1/2);
         y = y + h*s2;
         yout = [yout; y];
      end

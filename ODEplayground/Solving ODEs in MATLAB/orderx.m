function p = orderx(odex)
% ORDER  Experiment to determine order
% of ODE solver.
% Integrate 1/(1+t)^2 from 0 to 1.
% Exact value:

% Copyright 2014 - 2015 The MathWorks, Inc.

   vexact = 0.5;
   F = @(t,y) 1/(1+t)^2;
   t0 = 0;
   tfinal = 1;
   y0 = 0;
   h = 0.1
   yout = odex(F,t0,h,tfinal,y0);
   v1 = yout(end)
   h = h/2
   yout = odex(F,t0,h,tfinal,y0);
   v2 = yout(end)
   ratio = (v1 - vexact)/(v2 - vexact)
   p = round(log2(ratio));
end

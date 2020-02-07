function tumble
   % Angular momentum of tumbling box.
   % Strang, Differential Equations and Linear Algebra, pp. 176-178.
   % Alar Toomre, MIT

   % Copyright 2014 - 2015 The MathWorks, Inc.

   function ydot = momemtum(t,y,y0)
      ydot = [ y(2)*y(3)
            -2*y(1)*y(3)
               y(1)*y(2)];
   end

   function my_btn_down(h)
      % Ref: Mike Garrity, email.
      ax = ancestor(h,'axes');
      cp = get(ax,'CurrentPoint');
      l = cp(2,:) - cp(1,:);
      l = l / norm(l);
      o = cp(1,:);
      r = 1;
      c = [0 0 0];
      q = dot(l,(o-c))^2 - dot(o-c,o-c) + r^2;
      if q >= 0 
         d1 = -dot(l,(o-c)) + sqrt(q);
         d2 = -dot(l,(o-c)) - sqrt(q);      
         pt1 = o + d1*l;
         pt2 = o + d2*l;
         v = get(ax,'CameraTarget') - get(ax,'CameraPosition');
         if dot(v,l) < 0
            y0 = pt1';
         else
            y0 = pt2';
         end
         line(y0(1),y0(2),y0(3),'marker','.','color','k','markersize',8)
         [t,y] = ode45(@momemtum,[0,100],y0,opt,y0);
         line(y(:,1),y(:,2),y(:,3),'linestyle','none','marker','.','color','k','markersize',4)
      end
   end

   function [val,isterm,dir] = gstop(t,y,y0)
      % Event function for periodicity
      % See C. Moler, Numerical Computing with MATLAB, Section 7.2.
      d = y - y0;
      v = momemtum(t,y);
      val = d'*v;
      isterm = 1;
      dir = 1;
   end

   opt = odeset('RelTol',1.e-6,'events',@gstop);
   clf
   [X,Y,Z] = sphere(36);
   h = surface(X,Y,Z);
   view(3)
   shading interp
   axis equal
   axis vis3d
   grid on
   set(gca,'xdir','rev','ydir','rev')
   dkblue = [0 0 .5];
   magenta = [.75 0 .75];
   line(0,0,1.01,'marker','.','color',dkblue,'markersize',20)
   text(0,0,1.1,'Z','color',magenta,'fontsize',16)
   line(0,1.01,0,'marker','.','color',dkblue,'markersize',20)
   text(0,1.1,0,'Y','color',magenta,'fontsize',16)
   line(1.01,0,0,'marker','.','color',dkblue,'markersize',20)
   text(1.1,0,0,'X','color',magenta,'fontsize',16)
   h.ButtonDownFcn = @(h,~) my_btn_down(h);
end

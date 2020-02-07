function predprey(action)
% PREDPREY  Predator-prey gui.
% Drag the red dot to change equilibrium point.
% Drag the blue-green dot to change the initial conditions.

%   Copyright 2013 - 2015 The MathWorks, Inc.

   % Default parameters.

   mu = [300 200]';     % Equilibrium.
   eta = [400 100]';    % Initial conditions.

   % Predator-prey ode

   function ydot = ppode(t,y);
      ydot = [(1-y(2)/mu(2))*y(1);
             -(1-y(1)/mu(1))*y(2)];
   end
  
   % Switchyard.

   if nargin == 0
      action = 'init';
   end
   switch action
      case 'init'
         initialize_graphics
      case 'down'
         locate_dot
         return
      case 'move'
         move_dot
      case 'up'
         free_dot
   end

   % Solve ode.

   [mu, eta] = get_parameters;
   opts = odeset('reltol',1.e-8,'events',@pitstop);
   [t,y,te] = ode45(@ppode,[0 100],eta,opts);

   % Update the plots.

   subplot1(y,action)
   subplot2(t,y,te,action)

   % ----------------------------------

   function [g,isterm,dir] = pitstop(t,y)
      % Event function called by the ode solver.
      % Terminate when y returns to the point where its angle
      % with mu is the same as the angle between eta and mu.
      sig = sign(eta(1)-mu(1)+realmin);
      theta1 = atan2(sig*(y(2)-mu(2)),sig*(y(1)-mu(1)));
      theta0 = atan2(sig*(eta(2)-mu(2)),sig*(eta(1)-mu(1)));
      g = theta1 - theta0;
      isterm = t > 1;
      dir = 1;
   end

   % ----------------------------------

   function initialize_graphics
      % Set up two subplots, buttons, dots and empty plots.
      clf
      shg
      set(gcf,'menubar','none','numbertitle','off','name','Predprey', ...
          'units','normal','pos',[.25 .125 .50 .75])
      subplot(2,1,1)
      plot(0,0,'-','color','black');
      line(mu(1),mu(2),'marker','.','markersize',24,'color',[1 0 0]);
      line(eta(1),eta(2),'marker','.','markersize',24,'color',[0 1/2 1/2]);
      xlabel('prey')
      ylabel('predator')
      title('Drag either dot')
      subplot(2,1,2)
      plot(0,[0 0]);
      line([0 0],[0 0],'color','black');
      line([0 0],[0 0],'color','black');
      xlabel('time')
      legend('prey','predator','period','location','northwest')
      set(gcf,'windowbuttondownfcn','predprey(''down'')', ...
              'windowbuttonmotionfcn','predprey(''move'')', ...
              'windowbuttonupfcn','predprey(''up'')')
      set(gcf,'userdata',[])
   end

   % ----------------------------------

   function locate_dot
      % Find if the mouse is selecting one of the dots.
      point = get(gca,'currentpoint');
      h = get(gca,'children');
      y1 = get(h(1:2),'xdata');
      y2 = get(h(1:2),'ydata');
      d = abs([y1{:}]'-point(1,1)) + abs([y2{:}]'-point(1,2));
      k = min(find(d == min(d)));
      tol = .025*max(abs(axis));
      if d(k) < tol
         set(gcf,'userdata',h(k))
      else
         set(gcf,'userdata',[])
      end
   end

   % ----------------------------------

   function move_dot
      % Move the selected dot to a new position.
      point = abs(get(gca,'currentpoint'));
      hit = get(gcf,'userdata');
      if ~isempty(hit)
         set(hit,'xdata',point(1,1),'ydata',point(1,2))
      end
   end

   % ----------------------------------

   function free_dot
      % Deselect the dot.
      set(gcf,'userdata',[])
   end

   % ----------------------------------

   function [mu,eta] = get_parameters
      % Obtain mu and eta from the two dots.
      subplot(2,1,1);
      h = get(gca,'children');
      mu = [get(h(2),'xdata') get(h(2),'ydata')]';
      eta = [get(h(1),'xdata') get(h(1),'ydata')]';
   end

   % ----------------------------------

   function subplot1(y,action)
      % Redraw the phase plane plot and perhaps rescale.
      subplot(2,1,1)
      h = get(gca,'children');
      set(h(3),'xdata',y(:,1),'ydata',y(:,2));
      if ~isequal(action,'move')
         y1max = max(max(y(:,1)),mu(1));
         y2max = max(max(y(:,2)),mu(2));
         axis([0 1.5*y1max 0 1.5*y2max])
      end
   end

   % ----------------------------------

   function subplot2(t,y,te,action)
      % Redraw the time plots, period line, and perhaps rescale.
      subplot(2,1,2)
      if length(te)==0 || te(end) < 1.e-6
         pit = 2*pi;
      else
         pit = te(end);
      end
      h = get(gca,'children');
      ymax = max(max(y));
      t = [t; t+pit; t+2*pit];
      y = [y; y; y];
      set(h(4),'xdata',t,'ydata',y(:,1));
      set(h(3),'xdata',t,'ydata',y(:,2));
      set(h(2),'xdata',[pit pit],'ydata',[0 3*ymax]);
      set(h(1),'xdata',[2*pit 2*pit],'ydata',[0 3*ymax]);
      set(gca,'xtick',[0 pit 2*pit])
      if ~isequal(action,'move')
         axis([0 2.5*pit 0 1.5*ymax])
      end
      subplot(2,1,1)
   end
end

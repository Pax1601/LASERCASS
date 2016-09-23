function [NODES_U, NODES_L] = load_airfoil(foil)
%
  STEP = 50;
  EP = [0:1/STEP:1]';
% clear extension
  dotpos = find(foil=='.');
  if ~isempty(dotpos)
    foil = foil(1:dotpos(end)-1);
  end
  LOAD = 1;
% check if 4 digits
  if (~isempty(str2num(foil)))
      if (length(foil) == 4)
        m = str2num(foil(1))/100;	% gives first NACA-4 number
        p = str2num(foil(2))/10;	  % gives second NACA-4 number
        t = str2num(foil(3:4))/100;
	      for i=1:length(EP)
          if EP(i) < p
            camb(i,1)=(m*EP(i)/(p^2)*(2*p-EP(i)));  
          else
            camb(i,1)=m*(1-EP(1))/((1-p)^2)*(1 + EP(i)-2*p);  
          end
        end
        yt = t./0.2 .* (0.2969.*sqrt(EP) - 0.1260.* EP - 0.3516.*(EP).^2 + 0.2843 .*EP.^3 - 0.1015.*EP.^4);
        theta = atan(camb);
        Xu = EP - yt .* sin(theta);
        Yu = camb + yt .* cos(theta);
        Xl = EP + yt .* sin(theta);
        Yl = camb - yt .* cos(theta);
        Nu = length(EP); Nl = Nu; 
        A = [Nu Nu]; 
        A = [A; Xu, Yu; Xl, Yl];
        LOAD = 0;
      end
  elseif (LOAD==1)
  % check if file available
    if (exist(strcat(foil, '.dat'), 'file'))
      foil = strcat(foil, '.dat');
    elseif (exist(strcat(foil, '.DAT'), 'file'))
      foil = strcat(foil, '.DAT');
    else
      msg = ['Unable to find file for airfoil ', foil];
      error(msg);
    end
  %
    A = load(char(foil));
  % Take the number of data points in the data file
    Nu = A(1,1); % for the upper surface
    Nl = A(1,2); % for the lower surface
  %
  % check if format is ok
    if (Nu + Nl ~= size(A,1)-1)
      errmsg = ['Airfoil file ', char(foil), ' has no upper and lower points declaration at first line or wrong values given.'];
      error(errmsg);        
    end
  %
  end
  xup = A(2:Nu+1,1);
  yup = A(2:Nu+1,2);
  xdw = A(Nu+2:end,1);
  ydw = A(Nu+2:end,2);
  [xup, index] = sort(xup);
  yup = yup(index);
  [xdw, index] = sort(xdw);
  ydw = ydw(index);
  if (Nu ~= Nl)
    % determine missing points
    X1 = setdiff(xdw, xup);
    X2 = setdiff(xup, xdw); 
    yup = [yup; interp1(xup, yup, X1, 'cubic')];
    ydw = [ydw; interp1(xdw, ydw, X2, 'cubic')];
    xup = [xup; X1];         
    xdw = [xdw; X2];
    [xup, index] = sort(xup);
    yup = yup(index);
    [xdw, index] = sort(xdw);
    ydw = ydw(index);
  end
  %Upper surface
  Xu = xup/(xup(end) - xup(1));
  Yu = yup/(xup(end) - xup(1));
  % Lower surface
  Xl = xdw/(xdw(end) - xdw(1));
  Yl = ydw/(xdw(end) - xdw(1));
  %
  
  np = length(EP);
  Yu = interp1(Xu, Yu, EP, 'cubic');
  Yl = interp1(Xl, Yl, EP, 'cubic');
  NODES_U = zeros(np-1, 2);
  NODES_L = zeros(np-1, 2);

  NODES_U = [EP, Yu];
  NODES_L = [EP, Yl];
  %
%  figure(1); close; figure(1);
%  plot(EP, Yu,'o')
%  hold on
%  plot(EP, Yl,'o')
%  plot(CENTR_U(:,1),CENTR_U(:,2),'ro')
%  plot(CENTR_L(:,1),CENTR_L(:,2),'ro')

end
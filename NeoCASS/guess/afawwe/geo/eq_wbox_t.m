%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
%
% Politecnico di Milano, Dipartimento di Ingegneria Aerospaziale
% Via La Masa 34, 20156 Milano - ITALY
% 
% This file is part of NeoCASS Software (www.neocass.org)
%
% NeoCASS is free software; you can redistribute it and/or
% modify it under the terms of the GNU General Public
% License as published by the Free Software Foundation;
% either version 2, or (at your option) any later version.
%
% NeoCASS is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied
% warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
% PURPOSE.  See the GNU General Public License for more
% details.
%
% You should have received a copy of the GNU General Public
% License along with NeoCASS; see the file GNU GENERAL 
% PUBLIC LICENSE.TXT.  If not, write to the Free Software 
% Foundation, 59 Temple Place -Suite 330, Boston, MA
% 02111-1307, USA.
%
%
%*******************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
% function tmax_eq = eq_wbox_t(foil, spar_frac)
% find the equivalent wing box thickness, given aerodynamic profile and spar fraction
% the profile is supposed to have unitary chord
%
function tmax_eq = eq_wbox_t(foil, spar_frac)
%
  STEP = 50;
  EP = [0:1/STEP:1]';
  spar_frac = sort(spar_frac);
  if (spar_frac(1)<0)
    error('First spar fraction must be higher than zero.')
  end
  if (spar_frac(2)>1)
    error('Second spar fraction must be lower than one.')
  end
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
      else
        LOAD = 1;
      end
  end
  if (LOAD==1)
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
  
  P1 = spar_frac(1);
  P2 = spar_frac(2);
  i1 = find(EP>P1);
  i2 = find(EP<P2);
  ind = intersect(i1,i2);
  EP = [P1; EP(ind); P2];
  np = length(EP);
  Yu = interp1(Xu, Yu, EP, 'cubic');
  Yl = interp1(Xl, Yl, EP, 'cubic');
  NODES_U = zeros(np-1, 2);
  NODES_L = zeros(np-1, 2);
  CENTR_U = zeros(np-1, 2);
  CENTR_L = zeros(np-1, 2);
  Lpu = zeros(np-1, 1);
  Lpl = zeros(np-1, 1);

  NODES_U = [EP, Yu];
  NODES_L = [EP, Yl];
  CENTR_U(:,1) = 0.5.*[EP(1:end-1)+EP(2:end)];
  CENTR_U(:,2) = 0.5.*[Yu(1:end-1)+Yu(2:end)];
  CENTR_L(:,1) = CENTR_U(:,1);
  CENTR_L(:,2) = 0.5.*[Yl(1:end-1)+Yl(2:end)];
  for k=1:length(EP)-1
    Lpu(k) = norm(NODES_U(k,:) - NODES_U(k+1,:));
    Lpl(k) = norm(NODES_L(k,:) - NODES_L(k+1,:));
  end
  Lputot = sum(Lpu);
  Lpltot = sum(Lpl);
  % section CG
  YO = (sum(Lpu.*CENTR_U(:,2)) + sum(Lpl.*CENTR_L(:,2))) / (Lputot + Lpltot);
  % max distance from CG
  yumax = max(CENTR_U(:,2) - YO);
  ylmax = min(CENTR_L(:,2) - YO);
  %
  tmax_eq = sum( Lpu.* (CENTR_U(:,2) - YO).^2 )/(yumax * Lputot) -  sum( Lpl.* (CENTR_L(:,2) - YO).^2 )/(ylmax * Lpltot);
  %
%  figure(1); close; figure(1);
%  plot(EP, Yu,'o')
%  hold on
%  plot(EP, Yl,'o')
%  plot(CENTR_U(:,1),CENTR_U(:,2),'ro')
%  plot(CENTR_L(:,1),CENTR_L(:,2),'ro')

end

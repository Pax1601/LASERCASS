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
%--------------------------------------------------------------------------------------------------
%
%
%
%
% Called by:    Stick_Points.m
%
% Calls:        rot3D.m
%
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Mofified by Travaglini 19/11/2009, a new point on fuselage is defined,
% but vtail is not extended
function [stick, geo] = Stick_Points_Vert(aircraft, stick, geo)

% Elastic line points
stick.ptos.vert = geo.vtail.C2;
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        stick.ptos.vert(2,:) = geo.vtail.C2(2,:)+aircraft.Vertical_tail.Twin_tail_span*geo.wing.b/2;
    end
% Panel points
stick.ptospanel.vert = geo.vtail.PANE;
    if isequal(aircraft.Vertical_tail.Twin_tail, 1)
        stick.ptospanel.vert(2,:) = geo.vtail.PANE(2,:)+aircraft.Vertical_tail.Twin_tail_span*geo.wing.b/2;
    end
%
% Cut double poins generated in geometry modulus
if length(geo.vtail.CAERO1.n) > 1
    stick.ptos.vert(:,3:2:end-1) = [];
end
%
stick.part.vert = [];
if aircraft.Vertical_tail.Twin_tail
  LAB = 701;
else
  LAB = 301;
end

%------------------------------------------------------------------------------------------------------------
% sector 1
if (geo.vtail.span_inboard ~= 0)
    
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.vert)
        stick.IDCAERO1.vert = LAB;
    else
        stick.IDCAERO1.vert = [stick.IDCAERO1.vert; stick.IDCAERO1.vert(end)+1];
    end
    %
    stick.nx.vert = [stick.nx.vert; stick.nx.vert_inboard];
    stick.ny.vert = [stick.ny.vert; stick.ny.vert_inboard];
    stick.nx.sup_control.vert = [stick.nx.sup_control.vert; stick.nx.sup_control.vert_inboard];
    stick.nTOT.vert = [stick.nTOT.vert; stick.nx.vert_inboard*stick.ny.vert_inboard];
    stick.part.vert = [stick.part.vert, 1];
end
%
% sector 2
if (geo.vtail.span_outboard ~= 0)
    
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.vert)
        stick.IDCAERO1.vert = LAB+1;
    else
        stick.IDCAERO1.vert = [stick.IDCAERO1.vert; stick.IDCAERO1.vert(end)+1];
    end
    %
    stick.nx.vert = [stick.nx.vert; stick.nx.vert_outboard];
    stick.ny.vert = [stick.ny.vert; stick.ny.vert_outboard];
    stick.nx.sup_control.vert = [stick.nx.sup_control.vert; stick.nx.sup_control.vert_outboard];
    stick.nTOT.vert = [stick.nTOT.vert; stick.nx.vert_outboard*stick.ny.vert_outboard];
    stick.part.vert = [stick.part.vert, 2];
end
%
%------------------------------------------------------------------------------------------------------------
% Update total number of panels if some control surfaces are defined
stick.nTOT.vert = (stick.nx.vert + stick.nx.sup_control.vert).*stick.ny.vert;
%**************************************************************************
% Fuselage/VT connection establishment
if isequal(stick.model.fuse, 1) && isequal(aircraft.Vertical_tail.Twin_tail, 0)
    dist = zeros(size(stick.ptos.vert,2),1);
    for i = 1 : size(stick.ptos.vert,2)
       dist(i) = min(abs(stick.ptos.fuse(3,:) - stick.ptos.vert(3,i)));   
    end
    [dummy,indmin] = min(dist);
    VERTP = stick.ptos.vert(:,indmin);
    geo.vtail.link = VERTP;
    ptoz = interp1(stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), VERTP(1,1));
    
    %     if (ptoz ~= stick.ptos.vert(3,1))
    %
    ptoVertFuse = [VERTP(1,1); 0; ptoz];
    
    if isempty(find(abs(stick.ptos.fuse(1,:)-ptoVertFuse(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse,1))
        stick.ptos.fuse = [stick.ptos.fuse, ptoVertFuse];
    else
        stick.ptos.fuse(:,abs(stick.ptos.fuse(1,:)-ptoVertFuse(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse) = ptoVertFuse;
    end
    stick.link.vert = 'fuse';
    %
    
    %         geo.vtail.CAERO1.n = [1; geo.vtail.CAERO1.n];
    %         geo.vtail.CAERO1.n_coarse = [1; geo.vtail.CAERO1.n_coarse];
    
    %     else
    %         % fuselage connection point
    %         stick.ptos.fuse = [stick.ptos.fuse, stick.ptos.vert(:,1)];
    %     end
    
% elseif  aircraft.Vertical_tail.Twin_tail && aircraft.Tailbooms.present
%     stick.ptos.vert = [ptoVertFuse, stick.ptos.vert];
       %...
else 
    if isfield(aircraft,'Tailbooms')&&aircraft.Tailbooms.present
            if abs(stick.ptos.vert(2,1)-stick.ptos.tbooms(2,1))<=0.001*geo.wing.b
                stick.ptos.vert(2,:) = stick.ptos.tbooms(2,1)/stick.ptos.vert(2,1) * stick.ptos.vert(2,:);
                [dz,IND] = min(abs(stick.ptos.vert(3,:)-stick.ptos.tbooms(3,1)));
                [dx,IND2]= min(abs(stick.ptos.vert(1,IND)-stick.ptos.tbooms(1,:)));
                if dx<=0.02*geo.wing.b && dz<=0.02*geo.wing.b
                    stick.ptos.tbooms(:,IND2) =  stick.ptos.vert(:,IND);
                    stick.vtail.DZ = stick.ptos.vert(3,IND)-geo.tbooms.z;
                else
                    stick.vtail.DZ = stick.ptos.vert(3,IND)-geo.tbooms.z;
                    if dx <= 0.2*geo.tbooms.bodl/geo.tbooms.CAERO1.n_coarse
                        stick.ptos.tbooms(1,IND2) = stick.ptos.vert(1,IND);
                    else
                        stick.ptos.tbooms = [stick.ptos.tbooms,[ stick.ptos.vert(1,IND);stick.ptos.tbooms(2:3,1)]];
                        [Y, J] = sort(stick.ptos.tbooms(1,:), 'ascend');
                        % Sort points
                        stick.ptos.tbooms(1,:) = Y;
                        stick.ptos.tbooms(2,:) = stick.ptos.tbooms(2,J);
                        stick.ptos.tbooms(3,:) = stick.ptos.tbooms(3,J);
                    end      
                end
                stick.link.vert = 'tbooms';
            end
    else
% check if twin tail can be connected to fuselage or wing through simple RBE2
      FDIST = realmax;
      WDIST = FDIST;
      if stick.model.fuse
        for i=1:size(stick.ptos.vert,2)
          [ind, dist] = find_nearest_master(stick.ptos.vert(:,i), stick.ptos.fuse);
          if (dist<FDIST)
            FDIST = dist;
          end
        end
      end
      if stick.model.winr
        for i=1:size(stick.ptos.vert,2)
          [ind, dist] = find_nearest_master(stick.ptos.vert(:,i), stick.ptos.winr);
          if (dist<WDIST)
            WDIST = dist;
          end
        end
      end
    if (FDIST<WDIST)
      stick.link.vert = 'fuse';
    else
      stick.link.vert = 'wing';
    end
  end

end

%**************************************************************************


% Force to zero y coordinate
% indzero = find( abs(stick.ptos.vert(2,:)) < 1e-15);
stick.ptos.vert(2,stick.ptos.vert(2,:)<= 1e-15) = 0.0;

% Sort points
if isequal(stick.model.fuse, 1)
    [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
    % Sort points
    stick.ptos.fuse(1,:) = Y;
    stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
    stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
end
%
% if isequal(stick.model.vert, 1)
%     [Y, J] = sort(stick.ptos.vert(3,:), 'ascend');
%     % Sort points
%     stick.ptos.vert(1,:) = stick.ptos.vert(1,J);
%     stick.ptos.vert(2,:) = stick.ptos.vert(2,J);
%     stick.ptos.vert(3,:) = Y;
% end

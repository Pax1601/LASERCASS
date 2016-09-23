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
%
% Called by:    Stick_Points.m
%
% Calls:        RotInc, rot3D.m
%
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080211      1.0     A. Da Ronch      Creation
%     091119      1.3.9   L. Travaglini    Modification
%
% Modified by Travaglini: the distribution of aero pannels on control
% surfaces is changed and fuselage is not translated
%*******************************************************************************
function [stick, geo] = Stick_Points_Wing(aircraft, stick, geo)

% Quarter-chord points
stick.ptos.winr = geo.wing.QC;
% Elastic line points
stick.ptos.winrC2 = geo.wing.C2;
% Panel points
stick.ptospanel.winr = geo.wing.PANE;
% stick.ptospanel.winl = stick.ptospanel.winr;
% stick.ptospanel.winl(2,:) = -stick.ptospanel.winl(2,:);
%
% Cut double poins generated in geometry modulus
if length(geo.wing.CAERO1.n) > 1
    stick.ptos.winr(:,3:2:end-1) = [];
    stick.ptos.winrC2(:,3:2:end-1) = [];
end
stick.part.winr = [];
stick.part.winrP = [];
%------------------------------------------------------------------------------------------------------------
% sector 1
if (geo.wing.span_inboard ~= 0)

    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.winr)
        stick.IDCAERO1.winr = 201;
    else
        stick.IDCAERO1.winr = [stick.IDCAERO1.winr; stick.IDCAERO1.winr(end)+1];
    end
    % Number of panels
    stick.nx.wing = [stick.nx.wing; stick.nx.wing_inboard];
    stick.ny.wing = [stick.ny.wing; stick.ny.wing_inboard];
    % Define total number of panels in the master patch
    stick.nTOT.wing = [stick.nTOT.wing; stick.nx.wing_inboard*stick.ny.wing_inboard];
    % Number of panels within control surface
    stick.nx.sup_control.wing = [stick.nx.sup_control.wing; stick.nx.sup_control.wing_inboard];
    stick.part.winr = [stick.part.winr, 2];
end
%
% sector 2
if (geo.wing.span_midboard ~= 0)

    % Define ID numbers for CAERO1 panels    
    if isempty(stick.IDCAERO1.winr)
        stick.IDCAERO1.winr = 202;
    else
        stick.IDCAERO1.winr = [stick.IDCAERO1.winr; stick.IDCAERO1.winr(end)+1];
    end
    % Number of panels
    stick.nx.wing = [stick.nx.wing; stick.nx.wing_midboard];
    stick.ny.wing = [stick.ny.wing; stick.ny.wing_midboard];    
    % Define total number of panels in the master patch
    stick.nTOT.wing = [stick.nTOT.wing; stick.nx.wing_midboard*stick.ny.wing_midboard];
    % Number of panels within control surface
    stick.nx.sup_control.wing = [stick.nx.sup_control.wing; stick.nx.sup_control.wing_midboard];    
    stick.part.winr = [stick.part.winr, 3];
end
%
% sector 3
if (geo.wing.span_outboard ~= 0)

    % Define ID numbers for CAERO1 panels    
    if isempty(stick.IDCAERO1.winr)
        stick.IDCAERO1.winr = 204;
    else
        stick.IDCAERO1.winr = [stick.IDCAERO1.winr; stick.IDCAERO1.winr(end)+2];
    end       
    % Number of panels
    stick.nx.wing = [stick.nx.wing; stick.nx.wing_outboard];
    stick.ny.wing = [stick.ny.wing; stick.ny.wing_outboard];    
    % Define total number of panels in the master patch
    stick.nTOT.wing = [stick.nTOT.wing; stick.nx.wing_outboard*stick.ny.wing_outboard];
    % Number of panels within control surface
    stick.nx.sup_control.wing = [stick.nx.sup_control.wing; stick.nx.sup_control.wing_outboard];    
    stick.part.winr = [stick.part.winr, 4];
end
%
% winglet
if (aircraft.winglet.present ==1) && aircraft.winglet.Span>0

    % Define ID numbers for CAERO1 panels    
    
    stick.IDCAERO1.winr = [stick.IDCAERO1.winr; 206];
          
    % Number of panels
    stick.nx.wing = [stick.nx.wing; stick.nx.wing_winglet];
    stick.ny.wing = [stick.ny.wing; stick.ny.wing_winglet];    
    % Define total number of panels in the master patch
    stick.nTOT.wing = [stick.nTOT.wing; stick.nx.wing_winglet*stick.ny.wing_winglet];
    % Number of panels within control surface
    stick.nx.sup_control.wing = [stick.nx.sup_control.wing; stick.nx.sup_control.wing_winglet];    
    stick.part.winr = [stick.part.winr, 5];
end
%
% carrythrough sector
if isequal(stick.model.fuse, 1)
    
    av_size(1) =  geo.wing.span_inboard / stick.ny.wing_inboard;
    av_size(2) =  geo.wing.span_midboard / stick.ny.wing_midboard;
    av_size(3) =  geo.wing.span_outboard / stick.ny.wing_outboard;
    av_size = ceil(abs(stick.ptos.winr(2,2))./av_size);
    index = find(av_size>0);
    stick.ny.wing_carryth = av_size(index(1));
    % Define ID numbers for CAERO1 panels    
    stick.IDCAERO1.winr = [200;stick.IDCAERO1.winr];
    % Number of panels
    stick.nx.wing = [stick.nx.wing(1)+stick.nx.sup_control.wing(1); stick.nx.wing];
    stick.ny.wing = [stick.ny.wing_carryth; stick.ny.wing];    
    % Define total number of panels in the master patch
    stick.nTOT.wing = [stick.nx.wing(1)*stick.ny.wing(1); stick.nTOT.wing];
    % Number of panels within control surface
    stick.nx.sup_control.wing = [0; stick.nx.sup_control.wing];   
    stick.part.winr = [1, stick.part.winr];
    
end
%------------------------------------------------------------------------------------------------------------
% Connection fuselage-wing
if isequal(stick.model.fuse, 1)
    
    % Interpolate for intersection point
    ptofwZ = interp1( stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), stick.ptos.winrC2(1,1) );
    ptofw = [stick.ptos.winrC2(1,1); stick.ptos.winrC2(2,1); ptofwZ];
    
    if isempty( find( ptofw(1)==stick.ptos.fuse(1,:) ,1) )
%         check the distance between ptofw and other points, if it is to
%         near to another one it will sobstitute by ptos
        if isempty(find(abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse,1))
            stick.ptos.fuse = [stick.ptos.fuse, ptofw];
        else
            stick.ptos.fuse(:,abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse) = ptofw;
        end
    end
    
    [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
    % Sort points
    stick.ptos.fuse(1,:) = Y;
    stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
    stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
    
%     % Translate fuselage points in Z-coordinate
%     dz = ptofwZ - stick.ptos.winrC2(3,1);
%     
%     if dz
%         stick.ptos.fuse(3,:) = stick.ptos.fuse(3,:) - dz;
%     end
%     
%     % Save the OFFSET for later use in CBAR card
%     stick.OFFSET.fuse = [0; 0; dz];
    stick.link.wing = 'fuse';
end
% Connection Tailbooms-wing
if isfield(aircraft,'Tailbooms')
   if aircraft.Tailbooms.present
      %In this case noone point is added in wing, but one point is added in tailbooms
      % first find point in wing
      Xwing = interp1(stick.ptos.winrC2(2,:),stick.ptos.winrC2(1,:),geo.tbooms.y,'linear');
      Zwing = interp1(stick.ptos.winrC2(2,:),stick.ptos.winrC2(3,:),geo.tbooms.y,'linear');
      [dy,IND] = min(abs(stick.ptos.winrC2(2,:) - geo.tbooms.y));
      [dx,IND2] = min(abs(Xwing - stick.ptos.tbooms(1,:)));
      if dy <= 0.001*geo.wing.b && dx <= 0.001*geo.wing.b && (abs(Zwing - stick.ptos.tbooms(3,IND2)))<= 0.001*geo.wing.b
          stick.ptos.tbooms(:,IND2) =  stick.ptos.winrC2(:,IND);
          stick.wing.X =Xwing;
      else
          stick.wing.X =Xwing;
          if dx <= 0.2*geo.tbooms.bodl/geo.tbooms.CAERO1.n_coarse
              stick.ptos.tbooms(1,IND2) = Xwing;
          else
              stick.ptos.tbooms = [stick.ptos.tbooms,[Xwing;stick.ptos.tbooms(2:3,1)]];
              [Y, J] = sort(stick.ptos.tbooms(1,:), 'ascend');
              % Sort points
              stick.ptos.tbooms(1,:) = Y;
              stick.ptos.tbooms(2,:) = stick.ptos.tbooms(2,J);
              stick.ptos.tbooms(3,:) = stick.ptos.tbooms(3,J);
          end
      end
      if ~isequal(stick.model.fuse, 1)
          stick.link.wing = 'tbooms';
      end
   end
end
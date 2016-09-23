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
%---------------------------------------------------------------------------------------------------------------------------
% UPDATED 2008/02/11
%
% Called by:    Stick_Points.m
%
% Calls:        RotInc, rot3D.m
%
%   <andreadr@kth.se>
%---------------------------------------------------------------------------------------------------------------------------
% Modified by Travaglini 19/11/2009, now this function describes wing2 and not canard
function [stick, geo] = Stick_Points_Wing2(aircraft, stick, geo)

% Quarter-chord points
stick.ptos.win2r = geo.wing2.QC;
% Elastic line points
stick.ptos.win2rC2 = geo.wing2.C2;
% Panel points
stick.ptospanel.win2r = geo.wing2.PANE;
stick.ptospanel.win2l = stick.ptospanel.win2r;
stick.ptospanel.win2l(2,:) = -stick.ptospanel.win2l(2,:);
%
% Cut double poins generated in geometry modulus
if length(geo.wing2.CAERO1.n) > 1
    stick.ptos.win2r(:,3:2:end-1) = [];
    stick.ptos.win2rC2(:,3:2:end-1) = [];
end

%------------------------------------------------------------------------------------------------------------
% sector 1
if (geo.wing2.span_inboard ~= 0)
    
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.win2r)
        stick.IDCAERO1.win2r = 601;
    else
        stick.IDCAERO1.win2r = [stick.IDCAERO1.win2r; stick.IDCAERO1.win2r(end)+1];
    end
    % Number of panels
    stick.nx.wing2 = [stick.nx.wing2; stick.nx.wing2_inboard];
    stick.ny.wing2 = [stick.ny.wing2; stick.ny.wing2_inboard];
    % Define total number of panels in the master patch
    stick.nTOT.wing2 = [stick.nTOT.wing2; stick.nx.wing2_inboard*stick.ny.wing2_inboard];
    % Number of panels within control surface
    stick.nx.sup_control.wing2 = [stick.nx.sup_control.wing2; stick.nx.sup_control.wing2_inboard];
    
end
%
% sector 2
if (geo.wing2.span_midboard ~= 0)
    
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.win2r)
        stick.IDCAERO1.win2r = 601;
    else
        stick.IDCAERO1.win2r = [stick.IDCAERO1.win2r; stick.IDCAERO1.win2r(end)+1];
    end
    % Number of panels
    stick.nx.wing2 = [stick.nx.wing2; stick.nx.wing2_midboard];
    stick.ny.wing2 = [stick.ny.wing2; stick.ny.wing2_midboard];
    % Define total number of panels in the master patch
    stick.nTOT.wing2 = [stick.nTOT.wing; stick.nx.wing2_midboard*stick.ny.wing2_midboard];
    % Number of panels within control surface
    stick.nx.sup_control.wing2 = [stick.nx.sup_control.wing2; stick.nx.sup_control.wing2_midboard];
    
end
%
% sector 3
if (geo.wing2.span_outboard ~= 0)
    
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.win2r)
        stick.IDCAERO1.win2r = 601;
    else
        stick.IDCAERO1.win2r = [stick.IDCAERO1.win2r; stick.IDCAERO1.win2r(end)+1];
    end
    % Number of panels
    stick.nx.wing2 = [stick.nx.wing2; stick.nx.wing2_outboard];
    stick.ny.wing2 = [stick.ny.wing2; stick.ny.wing2_outboard];
    % Define total number of panels in the master patch
    stick.nTOT.wing2 = [stick.nTOT.wing2; stick.nx.wing2_outboard*stick.ny.wing2_outboard];
    % Number of panels within control surface
    stick.nx.sup_control.wing2 = [stick.nx.sup_control.wing2; stick.nx.sup_control.wing2_outboard];
    
end
%
% carrythrough sector
if isequal(stick.model.fuse, 1)
    av_size(1) =  geo.wing2.span_inboard / stick.ny.wing2_inboard;
    av_size(2) =  geo.wing2.span_outboard / stick.ny.wing2_outboard;
    av_size = ceil(abs(stick.ptos.win2r(2,2))./av_size);    
    index = find(av_size>0);
    stick.ny.wing2_carryth = av_size(index(1));
    % Define ID numbers for CAERO1 panels
    stick.IDCAERO1.win2r = [600;stick.IDCAERO1.win2r];
    % Number of panels
    stick.nx.wing2 = [stick.nx.wing2(1)+stick.nx.sup_control.wing2(1); stick.nx.wing2];
    stick.ny.wing2 = [stick.ny.wing2_carryth; stick.ny.wing2];
    % Define total number of panels in the master patch
    stick.nTOT.wing2 = [stick.nx.wing(1)*stick.ny.wing2(1); stick.nTOT.wing2];
    % Number of panels within control surface
    stick.nx.sup_control.wing2 = [0; stick.nx.sup_control.wing2];
    
end
%------------------------------------------------------------------------------------------------------------


% Update total number of panels if some control surfaces are defined
for i = 1:length(geo.wing2.CAERO1.n)
    
    t = 1 + (i-1)*2;
    
    if (geo.wing2.CAERO1.sup_control.frc(t) > 0 && geo.wing2.CAERO1.sup_control.frc(t+1) > 0)
        
        if isequal(geo.wing2.CAERO1.sup_control.frs(i), 1.0)
            
            stick.nTOT.wing2(i) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))*stick.ny.wing2(i);
            
        else
            
            switch aircraft.wing1.aileron.position
                
                case 0 % add panels for one slave patch
                    stick.nTOT.wing2(i) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))*stick.ny.wing2(i) ;
                    stick.nTOT.wing2P = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))* ceil( stick.ny.wing2(i)*(1-geo.wing2.CAERO1.sup_control.frs(i)) ) ;
                case 2 % add panels for one master and one slave patch
                    stick.nTOT.wing2(i) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))*stick.ny.wing2(i) ;
                    stick.nTOT.wing2P(1) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))* ceil( 0.5*stick.ny.wing2(i)*(1-geo.wing2.CAERO1.sup_control.frs(i)) ) ;
                    stick.nTOT.wing2P(2) = stick.nTOT.wing2P(1);
                case 1 % add panels for one slave patch
                    stick.nTOT.wing2(i) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))*stick.ny.wing2(i) ;
                    stick.nTOT.wing2P = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))* ceil( stick.ny.wing2(i)*(1-geo.wing2.CAERO1.sup_control.frs(i)) ) ;
                otherwise
                    stick.nTOT.wing2(i) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))*stick.ny.wing2(i) ;
                    stick.nTOT.wing2P(1) = (stick.nx.wing2(i) + stick.nx.sup_control.wing2(i))* ceil( 0.5*stick.ny.wing2(i)*(1-geo.wing2.CAERO1.sup_control.frs(i)) ) ;
                    stick.nTOT.wing2P(2) = stick.nTOT.wing2P(1);
            end
            
        end
        
    end
    
end


% Connection fuselage-wing2
if isequal(stick.model.fuse, 1)
    
    % Interpolate for intersection point
    ptofwZ = interp1( stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), stick.ptos.win2rC2(1,1) );
    ptofw = [stick.ptos.win2rC2(1,1); stick.ptos.win2rC2(2,1); ptofwZ];
    
    if isempty( find( ptofw(1)==stick.ptos.fuse(1,:) ,1) )
        %         check the distance between ptofw and other points, if it is to
        %         near to another one it will sobstitute by ptos
        if isempty(find(abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse,1))
            stick.ptos.fuse = [stick.ptos.fuse, ptofw];
        else
            % The node is not sobsituted because it could be the
            % master node of fus-wing1 link
            Near = stick.ptos.fuse(1,:)-ptofw(1);
            [dummy,indDX] = min(abs(Near));
            stick.wing2.DX = Near(indDX);
            %             stick.ptos.fuse(:,abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.001*geo.fus.bodl) = ptofw;
        end
    end
    
    [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
    % Sort points
    stick.ptos.fuse(1,:) = Y;
    stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
    stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
    stick.link.wing2 = 'fuse';
    % Connection Tailbooms-wing
    if isfield(aircraft,'Tailbooms')
        if aircraft.Tailbooms.present
            %In this case noone point is added in wing, but one point is added in tailbooms
            % first find point in wing
            [dy,IND] = min(abs(stick.ptos.win2rC2(2,:) - geo.tbooms.y));
            [dx,IND2] = min(abs(stick.ptos.win2rC2(1,IND) - stick.ptos.tbooms(1,:)));
            if dy <= 0.001*geo.wing.b && dx <= 0.001*geo.wing.b && (abs(stick.ptos.win2rC2(3,IND) - stick.ptos.tbooms(3,IND2)))<= 0.001*geo.wing.b
                stick.ptos.tbooms(:,IND2) =  stick.ptos.win2rC2(:,IND);
                stick.wing2.DY =0;
            else
                stick.wing2.DY = stick.ptos.win2rC2(2,IND)-geo.tbooms.y;
                if dx <= 0.2*geo.tbooms.bodl/geo.tbooms.CAERO1.n_coarse
                    stick.ptos.tbooms(1,IND2) = stick.ptos.win2rC2(1,IND);
                else
                    stick.ptos.tbooms = [stick.ptos.tbooms,[stick.ptos.win2rC2(1,IND);stick.ptos.tbooms(2:3,1)]];
                    [Y, J] = sort(stick.ptos.tbooms(1,:), 'ascend');
                    % Sort points
                    stick.ptos.tbooms(1,:) = Y;
                    stick.ptos.tbooms(2,:) = stick.ptos.tbooms(2,J);
                    stick.ptos.tbooms(3,:) = stick.ptos.tbooms(3,J);
                end
            end
            if ~isequal(stick.model.fuse, 1)
                stick.link.wing2 = 'tbooms';
            end
        end
    end
end
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
% Travaglini 19/11/2009
function [stick, geo] = Stick_Points_Canard( stick, geo)

% Quarter-chord points 
stick.ptos.canr = geo.canard.QC;
% Elastic line points
stick.ptos.canrC2 = geo.canard.C2;
% Panel points
stick.ptospanel.canr = geo.canard.PANE;
stick.ptospanel.canl = stick.ptospanel.canr;
stick.ptospanel.canl(2,:) = -stick.ptospanel.canl(2,:);
%
% Cut double poins generated in geometry modulus
if length(geo.canard.CAERO1.n) > 1
    stick.ptos.canr(:,3:2:end-1) = [];
    stick.ptos.canrC2(:,3:2:end-1) = [];
end
stick.part.canr = [];
%------------------------------------------------------------------------------------------------------------
% sector 1
if (geo.canard.span_inboard ~= 0)
    stick.part.canr = [stick.part.canr, 2];
    % Define ID numbers for CAERO1 panels
    if isempty(stick.IDCAERO1.canr)
        stick.IDCAERO1.canr = 501;
    else
        stick.IDCAERO1.canr = [stick.IDCAERO1.canr; stick.IDCAERO1.canr(end)+1];
    end
    % Number of panels
    stick.nx.canr = [stick.nx.canr; stick.nx.canr_inboard];
    stick.ny.canr = [stick.ny.canr; stick.ny.canr_inboard];
    % Define total number of panels in the master patch
    stick.nTOT.canr = [stick.nTOT.canr; stick.nx.canr_inboard*stick.ny.canr_inboard];
    % Number of panels within control surface
    stick.nx.sup_control.canr = [stick.nx.sup_control.canr; stick.nx.sup_control.canard_inboard];

end
%
% sector 2
% if (geo.wing2.span_midboard ~= 0)
% 
%     % Define ID numbers for CAERO1 panels    
%     if isempty(stick.IDCAERO1.)
%         stick.IDCAERO1.canr = 500;
%     else
%         stick.IDCAERO1.canr = [stick.IDCAERO1.canr; stick.IDCAERO1.canr(end)+1];
%     end
%     % Number of panels
%     stick.nx.wing2 = [stick.nx.wing2; stick.nx.wing2_midboard];
%     stick.ny.wing2 = [stick.ny.wing2; stick.ny.wing2_midboard];    
%     % Define total number of panels in the master patch
%     stick.nTOT.wing2 = [stick.nTOT.wing2; stick.nx.wing2_midboard*stick.ny.wing2_midboard];
%     % Number of panels within control surface
%     stick.nx.sup_control.wing2 = [stick.nx.sup_control.wing2; stick.nx.sup_control.wing2_midboard];    
% 
% end

% sector 3
if (geo.canard.span_outboard ~= 0)
    stick.part.canr = [stick.part.canr, 3];
    % Define ID numbers for CAERO1 panels    
    if isempty(stick.IDCAERO1.canr)
        stick.IDCAERO1.canr = 502;
    else
        stick.IDCAERO1.canr = [stick.IDCAERO1.canr; stick.IDCAERO1.canr(end)+1];
    end       
    % Number of panels
    stick.nx.canr = [stick.nx.canr; stick.nx.canr_outboard];
    stick.ny.canr = [stick.ny.canr; stick.ny.canr_outboard];    
    % Define total number of panels in the master patch
    stick.nTOT.canr = [stick.nTOT.canr; stick.nx.canr_outboard*stick.ny.canr_outboard];
    % Number of panels within control surface
    stick.nx.sup_control.canr = [stick.nx.sup_control.canr; stick.nx.sup_control.canard_outboard];    

end
%
% carrythrough sector
if isequal(stick.model.fuse, 1)
    stick.part.canr = [1, stick.part.canr];
    av_size(1) =  geo.canard.span_inboard / stick.ny.canr_inboard;
    av_size(2) =  geo.canard.span_outboard / stick.ny.canr_outboard;
    av_size = ceil(abs(stick.ptos.canr(2,2))./av_size);    
    index = find(av_size>0);
    stick.ny.canr_carryth = av_size(index(1));
    % Define ID numbers for CAERO1 panels    
    stick.IDCAERO1.canr = [500;stick.IDCAERO1.canr];
    % Number of panels
    stick.nx.canr = [stick.nx.canr(1)+stick.nx.sup_control.canr(1); stick.nx.canr];
    stick.ny.canr = [stick.ny.canr_carryth; stick.ny.canr];    
    % Define total number of panels in the master patch
    stick.nTOT.canr = [stick.nx.canr(1)*stick.ny.canr(1); stick.nTOT.canr];
    % Number of panels within control surface
    stick.nx.sup_control.canr = [0; stick.nx.sup_control.canr];   
    
end
%------------------------------------------------------------------------------------------------------------
% Update total number of panels if some control surfaces are defined
stick.nTOT.canr = (stick.nx.canr + stick.nx.sup_control.canr).*stick.ny.canr;
% 
% % 
% % Connection fuselage-canard
if isequal(stick.model.fuse, 1)
    
    ptoz = interp1(stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), stick.ptos.canrC2(1,1));
    ptofw = [stick.ptos.canrC2(1,1); 0; ptoz];
    
    if isempty( find( ptofw(1)==stick.ptos.fuse(1,:) ,1) )
        %         check the distance between ptofw and other points, if it is to
        %         near to another one it will sobstitute by ptos
        if isempty(find(abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse,1))
            stick.ptos.fuse = [stick.ptos.fuse, ptofw];
        else
            % avoid the case when canard is very close to wing. this may cause the wing point to be deleted
            connect_nodes = stick.ptos.fuse(1,abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse);
            if connect_nodes ~= stick.ptos.winrC2(1,1)
              stick.ptos.fuse(:,abs(stick.ptos.fuse(1,:)-ptofw(1))<=0.2*geo.fus.bodl/geo.fus.CAERO1.n_coarse) = ptofw;
            else
              stick.ptos.fuse = [stick.ptos.fuse, ptofw];
            end
        end
    end
    
    [Y, J] = sort(stick.ptos.fuse(1,:), 'ascend');
    % Sort points
    stick.ptos.fuse(1,:) = Y;
    stick.ptos.fuse(2,:) = stick.ptos.fuse(2,J);
    stick.ptos.fuse(3,:) = stick.ptos.fuse(3,J);
    stick.link.canr = 'fuse';
end
% if isequal(stick.model.fuse, 1)
%     
%     % Interpolate for intersection point
%     ptofwZ = interp1( stick.ptos.fuse(1,:), stick.ptos.fuse(3,:), stick.ptos.canrC2(1,1) );
%     ptofw = [stick.ptos.canrC2(1,1); stick.ptos.canrC2(2,1); ptofwZ];
%     
%    
%     
%     
%     dz = ptofwZ - stick.ptos.canrC2(3,1);
%     ptofw = [stick.ptos.canrC2(1,1); stick.ptos.canrC2(2,1); ptofwZ-dz];
%     
%     
%     
%     if isempty( find( ptofw(1)==stick.ptos.fuse(1,:) ) )
%         stick.ptos.fuse = [stick.ptos.fuse, ptofw];
%     end
    
    % Translate fuselage points in Z-coordinate
%     dz = ptofwZ - stick.ptos.canrC2(3,1);
%     
%     if dz
%         stick.ptos.fuse(3,:) = stick.ptos.fuse(3,:) - dz;
%     end
%     
%     % Save the OFFSET for later use in CBAR card
%     stick.OFFSET.fuse = [0; 0; dz];
    
end

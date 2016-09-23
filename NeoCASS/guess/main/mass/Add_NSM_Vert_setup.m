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
% Compute paint non-structural mass using wetted surface fraction with
% respect total wetted surface.
%
%   Author: <andreadr@kth.se>
%
% MODIFICATIONS:
%     DATE        VERS     PROGRAMMER       DESCRIPTION
%     080722      1.0      A. Da Ronch      Creation
%     120502      2.1.237  Riccobene        Modification
%
%*******************************************************************************
function [str] = Add_NSM_Vert_setup(outf, pdcylin, aircraft, geo, stick, str)

nm = fieldnames(geo);
Swettot = 0;
for k = 1:numel(nm),
   
    if ~isempty(geo.(nm{k}))
       
        switch nm{k} 
            
            case {'fus', 'vtail'}
                Swettot = Swettot + sum(geo.(nm{k}).Swet);
            case {'wing', 'wing2', 'htail', 'canard', 'tbooms'}
                Swettot = Swettot + 2*sum(geo.(nm{k}).Swet);

        end
        
    end
    
end

% Wetted surface: interpolation over beam elements
Swet = zeros(length(stick.PID.vert), 1);

% dSwet = zeros(length(stick.PID.vert), 1);
indst = find( stick.nodes.vert(3,:)==geo.vtail.zLE );
% length(stick.PID.vert)
% length(geo.vtail.y_nodes(indst:end))
% indst
Swet(indst:end) = spline(geo.vtail.y_nodes_1_2_thick, geo.vtail.Swet, geo.vtail.y_nodes_1_2(indst:end)-geo.vtail.y_nodes_1_2(indst));
Swet(1:indst-1) = Swet(indst);
% dSwet = abs(diff(cumtrapz(Swet)));
dSwet = Swet;

% Node location
nodes = geo.vtail.y_nodes_1_2;

%**************************************************************************
% Paint
%
domain   = [0; geo.vtail.y_nodes(end)];
massPNT  = paint_weight(Swettot)*sum(geo.vtail.Swet)/Swettot;
mass_pnt = massNSM_distr(nodes, domain, massPNT, dSwet, Swet, stick.vtail.Lbeam);
if massPNT > 0.0
    % Save NSM
    str.vtail.NSM.dstr = str.vtail.NSM.dstr + mass_pnt;
    % Save separately
    for i = 1:length(mass_pnt)
        str.vtail.NSM.add.mass_pnt(2*i-1:2*i, 1) = mass_pnt(i);
    end
    str.vtail.NSM.add.MASS_PNT = mass_pnt;
    % VT paint
    paint = sum(mass_pnt.*stick.vtail.Lbeam);
    fprintf(outf, '\n\t\t- paint over vertical tail: %g kg.', paint);
    % Save in the total mass
    str.M = str.M + paint;
    str.vtail.paint = paint;
end
%
%**************************************************************************

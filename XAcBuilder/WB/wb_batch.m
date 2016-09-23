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
%**************************************************************************
%  SimSAC Project
%
%  NeoCASS
%  Next generation Conceptual Aero Structural Sizing  
%
%                      Sergio Ricci         <ricci@aero.polimi.it>
%                      Luca Cavagna         <cavagna@aero.polimi.it>
%                      Luca Riccobene       <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%**************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090303      2.1     L.Riccobene      Creation
%
%**************************************************************************
%
% function       aircraft = wb_batch(aircraft_xml, param_xml, flag)
%
%
%   DESCRIPTION:  Weight and Balance batch mode (version 2.0)
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                aircraft_xml   string     stores aircraft file name from Geo
%                                          module (*.xml);
%
%                param_xml      string     stores parameters needed by W&B
%                                          (*.xml);
%
%                flag           logical    [optional] if 'false' disable
%                                          parameter overwriting and leave
%                                          default values (all zeros);
%                                          could be omitted, i.e. flag =
%                                          true.
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                aircraft       struct     stores all aircraft data with
%                                          updated W&B fields
%         
%                
%    REFERENCES:
%
%**************************************************************************

function aircraft = wb_batch(aircraft_xml, param_xml, flag)

% Print to video WB version
vname = wb_version;
fprintf([' - ', vname, '\n']);

% By default overwrite mode is on
if nargin < 3
    flag = true;
end

% Load Geo xml file and add all necessary fields (without overwriting)
s = neocass_xmlwrapper(aircraft_xml);

% Init
aircraft = struct();

% Merge models only if file 'aircraft_xml' exists
if ~isempty(s)
    
    % Add parameters' default values (all zeros)
    wb_init = wb_struct_init();
    s = mergestruct2(s, wb_init, flag);
    
    % Load parameter xml file
    p = neocass_xmlwrapper(param_xml);
    
    % Same as before
    if isempty(p)
        return
    end
    
    % Merge structs in one (flag = true set overwrite mode: each common field
    % will be overwritten by parameter values)
    in = mergestruct2(s, p, flag);
    
    % Run WB
    aircraft = weight_xml(in);
    
end
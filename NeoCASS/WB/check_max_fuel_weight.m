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

function out = check_max_fuel_weight(aircraft)
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
% function       aircraft = check_max_fuel_weight(aircraft)
%
%
%   DESCRIPTION:   Check user specified fuel weight consistency with
%                  maximum allowable fuel weight stored in
%                  wing/central/auxiliary tanks. 
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                  
%                aircraft       struct     coming from Geo module
%                                          
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                out            struct     if all data are consistent out =
%                                          aircraft while, if any value
%                                          exceeds computed limits, out
%                                          fields are set conventionally to
%                                          these maximal value and a
%                                          warning message is issued.  
%         
%                
%    REFERENCES:
%
%**************************************************************************
%

out = aircraft;

% Maximum fuel weight estimate in wing/central/auxiliary tanks [Kg]
MFUWEI(1) = aircraft.fuel.max_weight_wing;
MFUWEI(2) = aircraft.fuel.max_weight_cent_wing_box;
MFUWEI(3) = aircraft.fuel.max_weight_aux;

% User specified fuel weight [Kg]
UFUWEI(1) = aircraft.weight_balance.Fuel.Maximum_fuel_in_wings;
UFUWEI(2) = aircraft.weight_balance.Fuel.Maximum_fuel_in_central_wingbox;
UFUWEI(3) = aircraft.weight_balance.Fuel.Maximum_fuel_in_auxiliary;

% Check values and overwrite if any exceeds limits
tank_name = {'Wing tank', 'Central tank', 'Auxiliary tank'};
tags = {'Maximum_fuel_in_wings', 'Maximum_fuel_in_central_wingbox', 'Maximum_fuel_in_auxiliary'};

fprintf('\tChecking fuel input...\n');

for i = 1:length(tank_name),
    
    if UFUWEI(i) > MFUWEI(i)
       if MFUWEI(i) < 0
            msg = [tank_name{i} ,' fuel weight negative, resetting to zero...'];
             warning('Warn:fuel_negative', msg, MFUWEI(i));
            out = assign_field_value(out, tags{i}, 0.);
        else
            msg = [tank_name{i} ,' fuel weight exceeds estimated limit of %8.1f [Kg]: setting the latter as actual value...'];
            warning('Warn:fuel_exceeds', msg, MFUWEI(i));
            out = assign_field_value(out, tags{i}, MFUWEI(i));
        end
    else
        fprintf(['\t- Fuel weight for ', tank_name{i},' within the limit of %8.1f [Kg]\n'], MFUWEI(i));
    end
    
end

fprintf('\tdone.\n');

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
%     140131       2.3     L.Riccobene      Creation
%
%**************************************************************************
%
% function        wb_clean(xml_filename, xml_filename_save)
%
%
%   DESCRIPTION:  Clean W&B struct.
%                 !Remark: if called with only an input it overwrites the
%                 original file.
%
%
%         INPUT: NAME               TYPE       DESCRIPTION
%
%                xml_filename       string     original *.xml file
%
%                xml_filename_save  string     destination *.xml file
%
%        OUTPUT: NAME               TYPE       DESCRIPTION
%
%                 
%
%    REFERENCES:
%
% See also fix_struct, find_imagORNaN, extract_field_value,
%          assign_field_value, recfieldnames, mergestruct2
%
%**************************************************************************
function wb_clean(xml_filename, xml_filename_save)

if nargin < 2
    % Attention: overwrites!
    xml_filename_save = xml_filename;
end

% Load file
aircraft = neocass_xmlwrapper(xml_filename);

if ~isempty(aircraft)
    
    % Check W&B field
    wb_field = 'weight_balance';
    
    if isfield(aircraft, wb_field) && ~isempty(aircraft.(wb_field))
        
        % Wipe out all fields
        aircraft.(wb_field) = [];
        
        % Still there can be NaN(s) or complex values
        if find_imagORNaN(aircraft, false) > 0
            aircraft = fix_struct(aircraft, false);
        end
        
        % Save the file
        neocass_xmlunwrapper(xml_filename_save, aircraft);
        
    end
    
else
    fprintf(' ## Couldn''t find %s file.\n', xml_filename);
end


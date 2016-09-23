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
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     090303      2.1     L.Riccobene      Creation
%
%**************************************************************************
%
% function        field_value = extract_field_value(s, name)
%
%
%   DESCRIPTION:  Extract field value whose name matches 'name' input (if exist).
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                  s            struct     input struct
%
%                  name         string     field name
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                 field_value              value stored in name
%
%
% !Remark: it doesn't yet work well with nested struct arrays (extract
%          only the last element value)
%
%    REFERENCES:
%
% See also assign_field_value, mergestruct2, fix_struct, find_imagORNaN
%
%**************************************************************************
function field_value = extract_field_value(s, name, field_value)

if nargin < 3
    field_value = [];
end

fn = fieldnames(s);

for n = 1:length(fn),
    
    % Recurse
    
    % Sub structure
    subs = s.(fn{n});
    
    % Check if struct array
    dim = numel(subs);
    
    if isstruct(subs)
        
        % Recurse
        if dim == 1
            field_value = extract_field_value(subs, name, field_value);
        else
            for k = 1:dim,
                field_value = extract_field_value(subs(k), name, field_value);
            end
        end
        
    else
        
        % If c_name matches field name, extract value
        if strcmpi(fn{n}, name)
            field_value = s.(fn{n});
        end
        
    end
    
end

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
% function        out = extract_struct(s, names)
%
%
%   DESCRIPTION:   Extract substruct from struct 's' given a cell array of
%                  parameter names. 
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                  s            struct     input struct
%
%                  names        cell       array of strings
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                 out           struct     substruct of 's' whose fields
%                                          are those specified in 'names'
%                                          (if found) 
%
%
% !Remark: still unstable on nested struct.
%
%    REFERENCES:
%
% See also extract_field_value, assign_field_value, mergestruct2,
%          fix_struct, find_imagORNaN 
%
%**************************************************************************
function out = extract_struct(s, names)

out = [];
fn = recfieldnames(s);

% Check if 'names' is a cell
if iscell(names)
    param_index = sample_parameters(fn, names); 
else
    error('extract_struct:wrong_input', '"names" must be a cell array!');
end

if ~isempty(param_index)

    for n = 1:length(param_index),
        value = extract_field_value(s, regexpi(fn{param_index(n)}, '\<.(\w*)$', 'match'));
        eval(['out', fn{param_index(n)}(2:end), '=', mat2str(value),';']);
    end
    
end

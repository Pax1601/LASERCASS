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
% function        names = recfieldnames(s)
%
%
%   DESCRIPTION:  Rercursively extract field names and returns a cell array
%                 with struct path.
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                  s            struct     input struct
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                 names         cell       array which stores field names
%
%
%    REFERENCES:
%
% See also extract_field_value, assign_field_value, mergestruct2
%
%**************************************************************************
function names = recfieldnames(s, Name, names)

if nargin < 2
    Name = inputname(1);
    names = [];
end

fn = fieldnames(s);

for n = 1:length(fn),

    Namei = [Name '.' fn{n}];

    % Recurse
    % s(1): get first element if s.fn{n} is a struct array (otherwise too
    % many elements are passed to isstruct)
    if isstruct(s(1).(fn{n}))
        names = recfieldnames(s.(fn{n}), Namei, names);
    else
        % Save variable
        names{end+1, 1} = Namei;
    end

end
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

function param_index = sample_parameters(param_list, choice)
%--------------------------------------------------------------------------
% Returns parameters indices given two cell array: 'param_list', searching
% list, and 'choice', desidered parameters.
% Syntax:
%
%           param_index = sample_parameters(param_list, choice)
%
% !Remark: if more than one occurence of some elemnt in 'choice' is found
% the code returns only the first encountered.
%--------------------------------------------------------------------------

param_index = zeros(length(choice), 1);

% Extract last token starting index (parameter name)
expr = '\<.(\w*)$';
tmp = regexpi(param_list, expr, 'match', 'once');

% Cycle through choice elements
for n = 1:length(choice),
    
    % search for exact matching
    test = strmatch(choice{n}, tmp, 'exact');
    
    if ~isempty(test)
        l = length(test);
        if l > 1
            fprintf(' - (%2d) Occurences of parameter "%s" found, using only first one!\n', l, choice{n});
            param_index(n) = test(1);
        else
            param_index(n) = test;
        end
        fprintf('Parameter "%s" found!\n', choice{n});
    else
        fprintf('Parameter "%s" not found!\n', choice{n});
    end

end

% Erase zero element
param_index = param_index(param_index~=0);
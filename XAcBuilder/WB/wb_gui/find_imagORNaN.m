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
%     140131       2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function        count = find_imagORNaN(s, flag)
%
%
%   DESCRIPTION:  Perform an integrity check on the struct 's', looking for
%                 NaN(s) and complex number; it returns how many elements
%                 meet the aforementioned criteria (and print to video the
%                 complete tree)
%
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                  s            struct     input struct
%
%                  flag         logical    enable/disable video output
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                count          double     counter
%
%    REFERENCES:
%
% See also fix_struct, extract_field_value, assign_field_value,
%          recfieldnames, mergestruct2
%
%**************************************************************************
function count = find_imagORNaN(s, flag, count, level, arrayIndex)

if nargin < 4
    % Start counter
    count = 0;
    % Level is used to compose the full path to field
    level = '';
    % Default index (i.e. not an array)
    arrayIndex = 1;
end

fn = fieldnames(s);

for n = 1:length(fn),

    % Sub structure
    subs = s.(fn{n});
    
    % Check if struct array
    dim = numel(subs);
    
    if isstruct(subs)
        
        % Recurse
        if dim == 1
            count = find_imagORNaN(subs, flag, count, fn{n}, dim);
        else
            for k = 1:dim,
                count = find_imagORNaN(subs(k), flag, count, fn{n}, k);           
            end
        end
        
    else

        % Check if complex or NaN
        check_fcn = @(x) ~isreal(x) || any( isnan(x(:)) );
        
        % Treat the case of cell array
        if ~iscell(subs)
            field_check = check_fcn(subs);
        else
            field_check = cellfun(check_fcn, subs);
        end

        if any(field_check)
            
            % Video output
            if flag
                % Output message with the complete tree
                if arrayIndex > 1
                    fprintf(' (!) Field " %s " complex or NaN.\n', [level, '(', num2str(arrayIndex), ').', fn{n}]);
                else
                    fprintf(' (!) Field " %s " complex or NaN.\n', [level '.' fn{n}]);
                end
            end
            
            % Increment counter
            count = count + 1;
        end
            
    end

end

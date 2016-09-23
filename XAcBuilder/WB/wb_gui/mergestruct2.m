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
% function        m = mergestruct2(s, a, flag)
%
%
%   DESCRIPTION:  Merge struct 'a' inside 's' with the following rules:
%
%                 - the "core" struct is s;
%                 - fields present only in 'a' are added to 's';
%                 - common fields, i.e. same names, can be either
%                 overwritten ('a' fields rule) or preserved (a so called
%                 'safe-mode', which means 's' rules).
%
%         INPUT: NAME           TYPE       DESCRIPTION
%
%                  s            struct     main input struct
%
%                  a            struct     second input struct
%
%                flag           logical    [optional] enable/disable
%                                           overwriting common values
%                                           (default = false)
%
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%                 m             struct     updated struct
%
%
%    REFERENCES:
%
% See also extract_field_value, assign_field_value, recfieldnames,
%          fix_struct, find_imagORNaN
%
%**************************************************************************
function m = mergestruct2(s, a, flag)

% Safe mode
if nargin < 3
    flag = false;
end

fna = fieldnames(a);

% Cycle through 'a' field names
for n = 1:length(fna),

    if isfield(s, fna{n})

        % Recurse
        if isstruct(a.(fna{n}))
            s.(fna{n}) = mergestruct2(s.(fna{n}), a.(fna{n}), flag);
        else
            % Overwrite value
            if flag
               s.(fna{n}) = a.(fna{n}); 
            end
        end

    else

        % Add field
        s.(fna{n}) = a.(fna{n});

    end

end

m = s;

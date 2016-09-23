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
% function         sout = fix_struct(s, flag)
%
%
%   DESCRIPTION:  Try to "clean" input struct 's' from NaN(s) and complex
%                 numbers: in the first case the value is set to 0. while
%                 in the latter only the real part is taken.
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
%                sout          struct      updated
%
%    REFERENCES:
%
% See also find_imagORNaN, extract_field_value, assign_field_value,
%          recfieldnames, mergestruct2
%
%**************************************************************************
function sout = fix_struct(s, flag, level, arrayIndex)

if nargin < 3
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
            s(dim).(fn{n}) = fix_struct(subs, flag, fn{n}, dim);
        else
            for k = 1:dim,
                s.(fn{n})(k) = fix_struct(subs(k), flag, fn{n}, k);
            end
        end
        
    else

        % Check if complex or NaN
        check_fcn = @(x) ~isreal(x) || any( isnan(x(:)) );
        
        % Treat the case of cell array
        cell_test = ~iscell(subs);
        if cell_test
            field_check = check_fcn(subs);
        else
            field_check = cellfun(check_fcn, subs);
        end

        if any(field_check)
            
            % Repair
            s.(fn{n}) = FixField(subs);
            
            % Video output
            if flag
                if arrayIndex > 1
                    fprintf(' - Field " %s " fixed.\n', [level, '(', num2str(arrayIndex), ').', fn{n}]);
                else
                    fprintf(' - Field " %s " fixed.\n', [level '.' fn{n}]);
                end
            end
            
        end
            
    end

end

sout = s;


function ff = FixField(in)
% Try to fix NaNs or complex number

% Init
ff = in;

if ~isreal(in)
    ff = real(in);
end

if any(isnan(in))
   ff(isnan(in)) = 0.;
end





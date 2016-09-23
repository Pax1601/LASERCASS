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

function m = mergestruct(s, a)
%--------------------------------------------------------------------------
% Add struct 'a' in struct 's', adding fields when missing and saving common
% ones; returns a struct.
%--------------------------------------------------------------------------

fna = fieldnames(a);

% Cycle through 'a' field names
for n = 1:length(fna),

    if isfield(s, fna{n})

        %Recurse
        if isstruct(a.(fna{n}))
            s.(fna{n}) = mergestruct(s.(fna{n}), a.(fna{n}));
        end

    else

        %Add field
        s.(fna{n}) = a.(fna{n});

    end

end

m = s;

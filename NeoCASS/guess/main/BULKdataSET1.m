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

%
%--------------------------------------------------------------------------------------------------
% BULKdataTRIM.m defines parameters for TRIM
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%    SET1     ID     
% 
% 
% Called by:    writeSET12file.m
% 
% Calls:        cnvt2_8chs.m
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataSET1(fid, ID, IDnodes, IDslaves)

%--------------------------------------------------------------------------
% Define field 1: SET1
%
fprintf(fid, 'SET1    ');

%--------------------------------------------------------------------------
% Define field 2: ID
%
[IDstr] = cnvt2_8chs(ID);
fprintf(fid, '%c', IDstr);

%--------------------------------------------------------------------------
% Define field from 3 to ...
%

% First two fields are already used
field = 2;

% Create one single vector with all IDs
IDALL = [IDnodes; IDslaves];

for i = 1 : length(IDALL)

    % Write in this current card field
    field = field + 1;
    % Convert to string and write in the file
    [IDALLstr] = cnvt2_8chs(IDALL(i));
    fprintf(fid, '%c', IDALLstr);
    
    % If current field is the 9th and more values must be written, create new line
    if ((field/9 == 1) & (i ~= length(IDALL)))
        % Update field number for the next line, starting from 2nd field
        field = 1;
        % New line and 8 empty spaces
        fprintf(fid, '\n        ');
    end
    
end


fprintf(fid, '\n');



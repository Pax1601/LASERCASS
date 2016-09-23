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

function writeINCLUDE2file(fid, filename_trim)
%--------------------------------------------------------------------------------------------------
% INCLUDE card
% 
% Inputs:       fid, indicating file to write
%               filename_trim, file to include
% 
% Called by:    export_model_smartcad.m
% 
%   <degaspari@aero.polimi.it>
%--------------------------------------------------------------------------------------------------
%
line = ['INCLUDE ', filename_trim];

fprintf(fid, 'SOL 144');
fprintf(fid, '\n$');

fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fid, '$ GUESS/SMARTCAD trim interface automatically included by GUESS');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
fprintf(fid, '$\n');
fprintf(fid, '%s', line);
fprintf(fid, '\n$\n');

end



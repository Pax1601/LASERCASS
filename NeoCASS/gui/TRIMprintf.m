%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (C) 2008 - 2011 
% 
% Sergio Ricci (sergio.ricci@polimi.it)
% Alessandro De Gaspari (degaspari@aero.polimi.it)
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

function TRIMprintf(state_cell)
%
path = neoguiscratchpath(); gui_param = load(fullfile(path, 'neocass_gui_param.mat'));
lab = gui_param.solver.aeros.TRIM_LABELS;
labels = lab(:,1);
nlabels = length(labels);



if ~isempty(state_cell),  % values contains retained flight conditions
    nstates = size(state_cell, 1);
else
    nstates = 0;
end



if ~nstates,

    fprintf(1, '\n - No flight conditions defined for SMARTCAD trim analysis. \n\n');

else

    for i = 1:nstates,
        
        % state_mat = str2double(state_cell);
        
        fprintf(1, '\n - Flight state ID: %d. \n\n', str2num(state_cell{i, 1}));
        
        fprintf(1, '      %s %s\n', [labels{2}, ':', blanks(9-length(labels{2}))], state_cell{i, 2});
        
        for j = 3:nlabels,
            
            fprintf(1, '      %s %g\n', [labels{j}, ':', blanks(9-length(labels{j}))], str2num(state_cell{i, j}));
            
        end
        

    end

end



end
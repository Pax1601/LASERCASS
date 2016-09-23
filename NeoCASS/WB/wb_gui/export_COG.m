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

function res = export_COG(filename, parts, COG, Imat)
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
%     090303      2.0     L.Riccobene      Creation
%
%**************************************************************************
%
% function      res = export_COG(filename, parts, COG, Imat)
%
%
%   DESCRIPTION:  Export W&B results to Excel sheet.
%
%         INPUT: NAME           TYPE         DESCRIPTION
%                  
%                filename       char         output file name
%
%                parts          cell array   part names
%
%                COG            matrix       weight_balance.COG field
%
%                Imat           matrix       inertia matrix
%
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%                
%                res            logical    array, 1 means succesful
%                                          writing, 0 failed. 
%
%
%    REFERENCES:
%
%**************************************************************************

% Prepare format
label = {'PART N°', 'Part Name', 'X [m]', 'Y [m]', 'Z [m]', 'Mass [Kg]', ' ', 'Inertia Matrix [Kgm^2]'};
numbers = {'1', '2', '3', '4', '5', '6', '7', '8', '9', '10',...
    '11', '12', '13', '14', '15', '16', '17', '18', '19', '20',...
    '21', '22', '23', '24', '25', '26', '27', '28', '29', '30'};
res = true(length(label), 1);
res_label = {'label', 'numbers', 'parts', 'COG', 'inertia'};
sheet = 'WB Results';

% Write results
fprintf('\n\tExporting results to %s...', filename);
warning('off', 'MATLAB:xlswrite:AddSheet');
res(1) = xlswrite(filename, label, sheet, 'A1:H1');
res(2) = xlswrite(filename, numbers', sheet, 'A2:A31');
res(3) = xlswrite(filename, parts', sheet, 'B2:B31');
res(4) = xlswrite(filename, COG, sheet, 'C2:F31');
res(5) = xlswrite(filename, Imat, sheet, 'H2:J4');

for i = 1:length(res),
    if ~res(i)
        fprintf('\nFailed to export %s', res_label{i});
    end
end
fprintf(' done.\n');

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
%*******************************************************************************
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
%*******************************************************************************
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080224      1.0     L.Cavagna        Creation
%
%*******************************************************************************
%
% function export_eigsol_param(fp, method, point, cord)
%
%   DESCRIPTION: Appends EIGR card to SMARTCAD file
%
%         INPUT: NAME           TYPE       DESCRIPTION
%                fp             pointer    SMARTCAD file pointer
%                method         integer    normalization method
%                fmin           real       minimum frequency
%                fmax           real       maximum frequency
%                point          integer    node ID
%                cord           integer    generalized coordinate for node ID
%                                       
%        OUTPUT: NAME           TYPE       DESCRIPTION
%
%    REFERENCES:
%
%*******************************************************************************

function export_eigsol_param(fp, method, fmin, fmax, nd, point, cord)
%
FIELD = 8;
%
fprintf(fp, '\n$ Eigenvalue solver parameters');
fprintf(fp, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
%
str = num2str(fmin, '%7g'); fmin_str = [str, blanks(FIELD-length(str))];
str = num2str(fmax, '%7g'); fmax_str = [str, blanks(FIELD-length(str))];
str = num2str(nd, '%d'); nd_str = [str, blanks(FIELD-length(str))];
%
str = num2str(point, '%d'); point_str = [str, blanks(FIELD-length(str))];
str = num2str(cord, '%d'); cord_str = [str, blanks(FIELD-length(str))];
%
line = ['\nEIGR    1       ', blanks(FIELD), fmin_str, fmax_str, blanks(FIELD), nd_str];
fprintf(fp, line);       
%
switch (method)

  case 2 % MAX method

    line = ['\n', blanks(FIELD), 'MAX     ',]; 
    fprintf(fp, line);

  case 3 % POINT method

    line = ['\n', blanks(FIELD), 'POINT   ', point_str, cord_str]; 
    fprintf(fp, line);

  otherwise % default MASS method

    line = ['\n', blanks(FIELD), 'MASS    ',]; 
    fprintf(fp, line);

end

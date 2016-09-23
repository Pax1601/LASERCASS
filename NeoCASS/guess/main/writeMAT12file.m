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
%--------------------------------------------------------------------------------------------------------------
% writeMAT12file.m uses the function BULKdataMAT1.m to write material properties for linear isotropic materials
%
% Called by:    guess.m
%
% Calls:        BULKdataMAT1.m
%
% MODIFIED 2008-08-06
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------------------
% Modified by Travaglini 19/11/2009, adding canard
%             Cavagna    09/11/2011, export correctly material limits
%
function writeMAT12file(outf, fid, pdcylin, stick, aircraft)


fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10');
fprintf(fid, '\n$ Material definition');
fprintf(fid, '\n$-------2-------3-------4-------5-------6-------7-------8-------9-------10\n');
%
fprintf(outf, '\n\tExporting material properties...');
%
DEFAULT_NU = 0.3;
SIGMA_TAU = sqrt(3);
%--------------------------------------------------------------------------------------------------
% Fuselage
%--------------------------------------------------------------------------------------------------
if isequal(pdcylin.stick.model.fuse, 1)
    % MAT1 card
    ST = pdcylin.fus.fts;
    SC = pdcylin.fus.fcs;
    SS = mean([ST, SC]) / SIGMA_TAU;
    BULKdataMAT1(fid, stick.MAT1.fuse, pdcylin.fus.es, [], DEFAULT_NU, pdcylin.fus.ds, ST, SC, SS);
end
%--------------------------------------------------------------------------------------------------
% Wing
%--------------------------------------------------------------------------------------------------
if isequal(pdcylin.stick.model.winr, 1)
    % MAT1 card
    SS = pdcylin.wing.fcsw;
    ST = SIGMA_TAU * SS;
    SC = ST;
    BULKdataMAT1(fid, stick.MAT1.wing, pdcylin.wing.esw, [], DEFAULT_NU, pdcylin.wing.dsw, ST, SC, SS);
end
%--------------------------------------------------------------------------------------------------
% Vertical tail
%--------------------------------------------------------------------------------------------------
if isequal(pdcylin.stick.model.vert, 1)
    % MAT1 card
    SS = pdcylin.vtail.fcsw;
    ST = SIGMA_TAU * SS;
    SC = ST;
    BULKdataMAT1(fid, stick.MAT1.vert, pdcylin.vtail.esw, [], DEFAULT_NU, pdcylin.vtail.dsw, ST, SC, SS);
end
%--------------------------------------------------------------------------------------------------
% Horizontal tail
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.horr, 1)
    % MAT1 card
    SS = pdcylin.htail.fcsw;
    ST = SIGMA_TAU * SS;
    SC = ST;
    BULKdataMAT1(fid, stick.MAT1.hori, pdcylin.htail.esw, [], DEFAULT_NU, pdcylin.htail.dsw, ST, SC, SS);
end
%--------------------------------------------------------------------------------------------------
% Canard
%--------------------------------------------------------------------------
if isequal(pdcylin.stick.model.canr, 1)
    % MAT1 card
    SS = pdcylin.canard.fcsw;
    ST = SIGMA_TAU * SS;
    SC = ST;
    BULKdataMAT1(fid, stick.MAT1.canr, pdcylin.canard.esw, [], DEFAULT_NU, pdcylin.canard.dsw, ST, SC, SS);
end
%--------------------------------------------------------------------------
% Tbooms
%--------------------------------------------------------------------------
if isequal(stick.model.tboomsr, 1)
    % MAT1 card
    ST = pdcylin.tbooms.fts;
    SC = pdcylin.tbooms.fcs;
    SS = mean([ST, SC]) / SIGMA_TAU;
    BULKdataMAT1(fid, stick.MAT1.tbooms, pdcylin.tbooms.es, [], DEFAULT_NU, pdcylin.tbooms.ds, ST, SC, SS);
end
fprintf(outf, 'done.');

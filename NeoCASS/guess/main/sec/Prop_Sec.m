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
%                      Sergio Ricci             <ricci@aero.polimi.it>
%                      Luca Cavagna             <cavagna@aero.polimi.it>
%                      Alessandro De Gaspari    <degaspari@aero.polimi.it>
%                      Luca Riccobene           <riccobene@aero.polimi.it>
%
%  Department of Aerospace Engineering - Politecnico di Milano (DIAPM)
%  Warning: This code is released only to be used by SimSAC partners.
%  Any usage without an explicit authorization may be persecuted.
%
%*******************************************************************************
%
%   Author: <andreadr@kth.se>
%
% Extract section mechanical properties according to structural sizing
% performed. Two option are implemented to calculate torsional constant:
%   - Bredt formula;
%   - monocoque method;
%
% Called by:    guess_mod.m
% 
% Calls:        Prop_Sec_Fuse.m, Prop_Sec_Wing.m, Prop_Sec_Vert.m, Prop_Sec_Hori.m   
%
% MODIFICATIONS:
%     DATE        VERS    PROGRAMMER       DESCRIPTION
%     080723      1.0     A. Da Ronch      Creation
%     091119      1.3.9   L. Travaglini    Modification
%
% Cleaned by Travaglini
%*******************************************************************************
function str = Prop_Sec(fid, pdcylin, stick, geo, str,aircraft)

% Fuselage
if isequal(pdcylin.stick.model.fuse, 1)
    str = Prop_Sec_Fuse(geo, str, stick);
end

% Wing
if isequal(pdcylin.stick.model.winr, 1)
    str = Prop_Sec_Wing(fid, pdcylin, geo, str, stick); 
end

% Vertical tail
if isequal(pdcylin.stick.model.vert, 1)
    str = Prop_Sec_Vert(fid, pdcylin, geo, str, stick);
end

% Horizontal tail
if isequal(pdcylin.stick.model.horr, 1)
   str = Prop_Sec_Hori(fid, pdcylin, geo, str, stick);
end

% Canard
if isequal(pdcylin.stick.model.canr, 1)
   str = Prop_Sec_Canr(fid, pdcylin, geo, str, stick);
end

% Wing2
if isequal(pdcylin.stick.model.win2r, 1)
   str = Prop_Sec_Wing2(fid, pdcylin, geo, str, stick);
end

% Tailbooms
if isequal(pdcylin.stick.model.tboomsr, 1)
    str = Prop_Sec_Tbooms(geo, str);
end
 
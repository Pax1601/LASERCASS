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
% BULKdataMAT1.m writes fields necessary to define linear isotropic material
% 
% |    1    |    2    |    3    |    4    |    5    |    6    |    7    |    8    |    9    |   10    |
%    MAT1       MID        E         G        NU        RHO
%               ST         SC       SS 
% 
% Called by:    writeMAT12file.m
% 
% Calls:        cnvt2_8chs.m
% 
% MODIFIED 2008-08-06
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
function [] = BULKdataMAT1(fid, MID, E, G, NU, RHO, ST, SC, SS)

%--------------------------------------------------------------------------
% If Young's Modulus (E), Shear Modulus (G) and Poisson's Ratio are
% entered and they do not satisfy the relation G = E/(2*(1+NU)), a warning
% message is printed indicating that the isotropic relation has been
% violeted.
if (isempty(E)~=0 & isempty(G)~=0 & isempty(NU)~=0)
    if (abs(1 - E/(2*(1+NU)*G)) > 0.01)
        fprintf('\n\tISOTROPIC RELATION HAS BEEN VIOLETED.\n');
    end
end

%--------------------------------------------------------------------------
% Define field 1: MAT1
fprintf(fid, 'MAT1    ');

%--------------------------------------------------------------------------
% Define field 2: MID
[MIDstr] = cnvt2_8chs(MID);
fprintf(fid, '%c', MIDstr);

%--------------------------------------------------------------------------
% Define field 3: E
if isempty(E)
    fprintf(fid, '        ');    
else    
    [Estr] = cnvt2_8chs(E);
    fprintf(fid, '%c', Estr);    
end

%--------------------------------------------------------------------------
% Define field 4: G
if isempty(G)    
    fprintf(fid, '        ');    
else    
    [Gstr] = cnvt2_8chs(G);
    fprintf(fid, '%c', Gstr);    
end

%--------------------------------------------------------------------------
% Define field 5: NU
if isempty(NU)    
    fprintf(fid, '        ');    
else    
    [NUstr] = cnvt2_8chs(NU);
    fprintf(fid, '%c', NUstr);    
end

%--------------------------------------------------------------------------
% Define field 6: RHO
[RHOstr] = cnvt2_8chs(RHO);
fprintf(fid, '%c', RHOstr);

%--------------------------------------------------------------------------
% New line for second row
fprintf(fid, '\n');

%--------------------------------------------------------------------------
% Define field 1: empty
fprintf(fid, '        ');

%--------------------------------------------------------------------------
% Define field 2: ST
[STstr] = cnvt2_8chs(ST);
fprintf(fid, '%c', STstr);

%--------------------------------------------------------------------------
% Define field 2: SC
[SCstr] = cnvt2_8chs(SC);
fprintf(fid, '%c', SCstr);

%--------------------------------------------------------------------------
% Define field 3: SS
[SSstr] = cnvt2_8chs(SS);
fprintf(fid, '%c', SSstr);

%--------------------------------------------------------------------------
% New line for second row
fprintf(fid, '\n');

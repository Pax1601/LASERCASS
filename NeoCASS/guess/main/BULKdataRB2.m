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
function [] = BULKdataRB2(fid, IDRB0, IDN, DOF, Slave)


%--------------------------------------------------------------------------------------------------
% Define field 1: RB0
%--------------------------------------------------------------------------------------------------

fprintf(fid, 'RBE2    ');


%--------------------------------------------------------------------------------------------------
% Define field 2: IDRB0
%--------------------------------------------------------------------------------------------------

[IDRB0str] = cnvt2_8chs(IDRB0);
fprintf(fid, '%c', IDRB0str);


%--------------------------------------------------------------------------------------------------
% Define field 3: IDN
%--------------------------------------------------------------------------------------------------

[IDNstr] = cnvt2_8chs(IDN);
fprintf(fid, '%c', IDNstr);

fprintf(fid, '%s', cnvt2_8chs(DOF));
%--------------------------------------------------------------------------------------------------
% Define field 4: IDS1
%--------------------------------------------------------------------------------------------------
if length(Slave) ==1
[IDS1str] = cnvt2_8chs(Slave(1));
fprintf(fid, '%c', IDS1str);
else

%--------------------------------------------------------------------------------------------------
% Define field 5: IDS2
%--------------------------------------------------------------------------------------------------
[IDS1str] = cnvt2_8chs(Slave(1));
fprintf(fid, '%c', IDS1str);

[IDS2str] = cnvt2_8chs(Slave(2));
fprintf(fid, '%c', IDS2str);

end
%--------------------------------------------------------------------------------------------------
% New line
%--------------------------------------------------------------------------------------------------

fprintf(fid, '\n');
end
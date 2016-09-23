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

function [stick] = Stick_ID_Wing2(stick)
%--------------------------------------------------------------------------------------------------
% Define ID numbers for wing component
% 
% 
% Called by:    Stick_ID.m
% 
% Calls:  
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Modified by Travaglini, 19/11/2009 now this function describes wing2 and
% not canard
stick.ID.win2r = (6000:6000+(length(stick.nodes.win2rC2(1,:))-1))';
stick.ID.win2r_thick = (6000:6000+(length(stick.nodes.win2rC2_thick(1,:))-1))';
stick.MAT1.wing2 = 600;
stick.IDSET.win2r = 600;
if isequal(stick.model.win2l, 1)     % Left semi-wing is defined
  stick.ID.win2l = [stick.ID.win2r(1),stick.ID.win2r(end)+1:stick.ID.win2r(end)+(length(stick.ID.win2r)-1)]';
  stick.ID.win2l_thick =  [stick.ID.win2r_thick(1),stick.ID.win2r_thick(end)+1:stick.ID.win2r_thick(end)+(length(stick.ID.win2r_thick)-1)]';
  stick.IDSET.win2l = 650;
end
% Property Identification Number
PIDwing_str = 6000;
stick.PID.wing2 = (PIDwing_str : PIDwing_str + (length(stick.ID.win2r)-2))';
stick.PID.wing2_thick = (PIDwing_str : PIDwing_str + (length(stick.ID.win2r_thick)-2))';
% Element Identification Number
EIDwing_str = 6000;
stick.EID.win2r = (EIDwing_str : EIDwing_str + (length(stick.ID.win2r)-2))'; 
stick.EID.win2r_thick = (EIDwing_str : EIDwing_str + (length(stick.ID.win2r_thick)-2))';    
%
% Simmetry if applicable
if isequal(stick.model.symmXZ, 1)
    stick.EID.win2l = (stick.EID.win2r(end)+1 : stick.EID.win2r(end)+1 + (length(stick.ID.win2r)-2))';
end
if isequal(stick.model.symmXZ, 1)
    stick.EID.win2l_thick = (stick.EID.win2r_thick(end)+1 : stick.EID.win2r_thick(end)+1 + (length(stick.ID.win2r_thick)-2))';
end

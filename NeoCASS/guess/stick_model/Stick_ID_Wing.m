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

function [stick] = Stick_ID_Wing(stick)
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
% Modified by Travaglini, 19/11/2009 now wing1 and fuselage don't have common nodes
stick.ID.winr = (2000:2000+(length(stick.nodes.winrC2(1,:))-1))';
stick.ID.winr_thick = (2000:2000+(length(stick.nodes.winrC2_thick(1,:))-1))';
stick.MAT1.wing = 200;
stick.IDSET.winr = 200;
if isequal(stick.model.winl, 1)     % Left semi-wing is defined
  stick.ID.winl = [stick.ID.winr(1),stick.ID.winr(end)+1:stick.ID.winr(end)+(length(stick.ID.winr))-1]';
  stick.ID.winl_thick =  [stick.ID.winr_thick(1),stick.ID.winr_thick(end)+1:stick.ID.winr_thick(end)+(length(stick.ID.winr_thick))-1]';
  % Identification Number for SET1 card
  stick.IDSET.winl = 250;
end
% Property Identification Number
PIDwing_str = 2000;
stick.PID.wing = (PIDwing_str : PIDwing_str + (length(stick.ID.winr)-2))';
stick.PID.wing_thick = (PIDwing_str : PIDwing_str + (length(stick.ID.winr_thick)-2))';

% Element Identification Number
EIDwing_str = 2000;
stick.EID.winr = (EIDwing_str : EIDwing_str + (length(stick.ID.winr)-2))'; 
stick.EID.winr_thick = (EIDwing_str : EIDwing_str + (length(stick.ID.winr_thick)-2))';    
%
% Simmetry if applicable
if isequal(stick.model.symmXZ, 1)
    stick.EID.winl = (stick.EID.winr(end)+1 : stick.EID.winr(end)+1 + (length(stick.ID.winr)-2))';
end
if isequal(stick.model.symmXZ, 1)
    stick.EID.winl_thick = (stick.EID.winr_thick(end)+1 : stick.EID.winr_thick(end)+1 + (length(stick.ID.winr_thick)-2))';
end
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

function [stick] = Stick_ID_Canr(stick)
%--------------------------------------------------------------------------------------------------
% Define ID numbers for canard component
% 
% 
% Called by:    Stick_ID.m
% 
% Calls:  
% 
%   <andreadr@kth.se>
%--------------------------------------------------------------------------------------------------
% Modified by Travaglini, 19/11/2009
% Grid Identification Number %
% stick.ID.fuse(IndFW),stick.ID.fuse_thick(IndFW_thick),
stick.ID.canr = (5000:5000+(length(stick.nodes.canrC2(1,:))-1))';
stick.ID.canr_thick = (5000:5000+(length(stick.nodes.canrC2_thick(1,:))-1))';
stick.MAT1.canr = 500;
stick.IDSET.canr = 500;
if isequal(stick.model.canl, 1)     % Left semi-wing is defined
  stick.ID.canl = [stick.ID.canr(1),stick.ID.canr(end)+1:stick.ID.canr(end)+(length(stick.ID.canr))-1]';
  stick.ID.canl_thick =  [stick.ID.canr_thick(1),stick.ID.canr_thick(end)+1:stick.ID.canr_thick(end)+(length(stick.ID.canr_thick))-1]';
  stick.IDSET.canl = 550;
end
% Property Identification Number
PIDwing_str = 5000;
stick.PID.canr = (PIDwing_str : PIDwing_str + (length(stick.ID.canr)-2))';
stick.PID.canr_thick = (PIDwing_str : PIDwing_str + (length(stick.ID.canr_thick)-2))';
% Element Identification Number
EIDwing_str = 5000;
stick.EID.canr = (EIDwing_str : EIDwing_str + (length(stick.ID.canr)-2))'; 
stick.EID.canr_thick = (EIDwing_str : EIDwing_str + (length(stick.ID.canr_thick)-2))';    
%
% Simmetry if applicable
if isequal(stick.model.symmXZ, 1)
    stick.EID.canl = (stick.EID.canr(end)+1 : stick.EID.canr(end)+1 + (length(stick.ID.canr)-2))';
end

if isequal(stick.model.symmXZ, 1)
    stick.EID.canl_thick = (stick.EID.canr_thick(end)+1 : stick.EID.canr_thick(end)+1 + (length(stick.ID.canr_thick)-2))';
end
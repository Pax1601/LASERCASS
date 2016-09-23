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

function [stick] = Stick_ID_Tbooms(stick)
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
stick.ID.tboomsr = (7000:7000+(length(stick.nodes.tboomsr(1,:))-1))';
stick.ID.tboomsr_thick = (7000:7000+(length(stick.nodes.tboomsr_thick(1,:))-1))';
stick.MAT1.tbooms = 700;
stick.IDSET.tboomsr = 700;
if isequal(stick.model.tboomsl, 1)     % Left tbooms is defined
    stick.ID.tboomsl = (stick.ID.tboomsr(end)+1:stick.ID.tboomsr(end)+(length(stick.ID.tboomsr)))';
    stick.ID.tboomsl_thick =  (stick.ID.tboomsr_thick(end)+1:stick.ID.tboomsr_thick(end)+(length(stick.ID.tboomsr_thick)))';
    stick.IDSET.tboomsl = 750;
end
% Property Identification Number
PIDtbooms_str = 7000;
stick.PID.tbooms = (PIDtbooms_str : PIDtbooms_str + (length(stick.ID.tboomsr)-2))';
stick.PID.tbooms_thick = (PIDtbooms_str : PIDtbooms_str + (length(stick.ID.tboomsr_thick)-2))';
% Element Identification Number
EIDtbooms_str = 7000;
stick.EID.tboomsr = (EIDtbooms_str : EIDtbooms_str + (length(stick.ID.tboomsr)-2))'; 
stick.EID.tboomsr_thick = (EIDtbooms_str : EIDtbooms_str + (length(stick.ID.tboomsr_thick)-2))';    
%
% Simmetry if applicable
if isequal(stick.model.symmXZ, 1)
    stick.EID.tboomsl = (stick.EID.tboomsr(end)+1 : stick.EID.tboomsr(end)+1 + (length(stick.ID.tboomsr)-2))';
end
if isequal(stick.model.symmXZ, 1)
    stick.EID.tboomsl_thick = (stick.EID.tboomsr_thick(end)+1 : stick.EID.tboomsr_thick(end)+1 + (length(stick.ID.tboomsr_thick)-2))';
end
